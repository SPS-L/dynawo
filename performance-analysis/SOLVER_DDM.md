# SolverDDM — Single-Level Schur-Complement Domain Decomposition Solver for Dynaωo

## Executive Summary

This document is a design brainstorm for `SolverDDM`, a new Dynaωo solver that implements the **single-level Schur-complement domain decomposition method** (network vs. injectors) as described in the reference papers. The solver targets large-scale time-domain dynamic simulation with variable-step BDF-2 integration, OpenMP parallelism over injector sub-domains, and zero dependency on SUNDIALS IDA or KINSOL at runtime. It slots into `dynawo/sources/Solvers/` as a peer of `SolverIDA` and `SolverSIM`.

---

## 1. Background and Reference Method

The decomposition method implemented in `SolverDDM` is the **single-level Schur-complement DDM** described in the journal paper and Chapter 4 of the corresponding thesis. The algorithm performs a **topological star-shaped decomposition**: each injector (synchronous machine, load, motor, wind turbine, etc.) is one sub-domain, and the entire electric network is the central sub-domain. **Only this single-level decomposition is in scope** — the two-level extension (Chapter 5 of the thesis) that further decomposes the network itself is explicitly excluded.

The interface variables are fixed by physics:
- **Injector → network**: injected current at the connection bus (`x_ext_i`)
- **Network → injector**: terminal voltage of that bus (`V_ext`)

No METIS/Scotch partitioning is needed; the decomposition is read directly from the `DYNConnector` topology at `init()` time.

The reference method defines two parallel variants on top of the single-level decomposition. Both share the same injector-vs-network structure:

| Variant | Injector update strategy | Network solve | Parallelism |
|---|---|---|---|
| **(P)** Basic parallel | All injectors solved each iteration; global matrix update criterion | One KLU solve on reduced system | OpenMP over injectors |
| **(EP)** Enhanced parallel | Per-injector local convergence: converged sub-domains stop early; asynchronous matrix updates | One KLU solve on reduced system | OpenMP over injectors |

Variant (EP) provides additional **numerical acceleration** on top of the computational parallelism: injectors that converge early are skipped in subsequent iterations, reducing total work per time step. Both variants remain single-level and are in scope. The recommended implementation order is (P) first as a correctness baseline, then (EP) as the production target.

---

## 2. Solver Architecture and Location

### 2.1 Directory Layout

```
dynawo/sources/Solvers/
├── AlgebraicSolvers/          # SolverKINAlgRestoration (not used by SolverDDM)
├── Common/                    # Solver::Impl base class
├── FixedTimeStep/             # SolverSIM, SolverTRAP (reference for control flow)
├── VariableTimeStep/
│   └── SolverIDA/             # SolverIDA (reference for variable-step pattern)
└── DDM/                       # NEW
    ├── CMakeLists.txt
    ├── DYNSolverDDM.h
    ├── DYNSolverDDM.cpp
    ├── DYNSolverDDMFactory.cpp
    └── DYNSubDomainDDM.h/.cpp  # Per-injector sub-domain object
```

### 2.2 Class Hierarchy

`SolverDDM` inherits from `Solver::Impl` exactly as `SolverIDA` and `SolverCommonFixedTimeStep` do. It overrides the mandatory virtual interface:

```cpp
class SolverDDM : public Solver::Impl {
 public:
  const std::string& solverType() const override;
  void defineSpecificParameters() override;
  void setSolverSpecificParameters() override;
  void init(const std::shared_ptr<Model>& model, double t0, double tEnd) override;
  void calculateIC(double tEnd) override;
  void reinit() override;
  bool setupNewAlgRestoration(modeChangeType_t modeChangeType) override;

 protected:
  void solveStep(double tAim, double& tNxt) override;

 private:
  // BDF-2 history
  std::vector<double> yPrev_;        // y^n
  std::vector<double> yPrevPrev_;    // y^(n-1)
  std::vector<double> ySave_;        // snapshot before Newton attempts
  std::vector<double> vectorF_;      // residual buffer
  double hPrev_;                     // h_{n-1}
  bool   bdf1Bootstrap_;             // true: first step or post-reinit
  double alpha0_, alpha1_, beta1_;   // BDF-2 coefficients
  double cj_;                        // = alpha0_, passed to evalJt

  // Step-size control
  double hMin_, hMax_, hCurrent_, hNew_;
  double kReduceStep_;               // bounds clamp on LTE factor
  double relTol_, absTol_;
  int    maxNewtonTry_;

  // Sub-domains
  std::vector<SubDomainDDM> subDomains_;  // one per injector SubModel
  std::vector<int> networkVarIndices_;    // indices of network variables in global y

  // Jacobian management
  int    jacAge_;
  int    jacMaxAge_;
  bool   factorizationForced_;

  // Network KLU objects
  klu_symbolic* kluNetSym_;
  klu_numeric*  kluNetNum_;
  klu_common    kluCommon_;
  SparseMatrix  smNet_;              // assembled reduced network system

  std::vector<int> differentialVariablesIndices_;
};
```

### 2.3 Factory Registration

```cpp
// DYNSolverDDMFactory.cpp
extern "C" DYN::SolverFactory* getFactory() {
  return new DYN::SolverDDMFactory();
}
extern "C" DYN::Solver* DYN::SolverDDMFactory::create() const {
  return new DYN::SolverDDM();
}
```

```cmake
# CMakeLists.txt (DDM/)
dynawo_add_library(dynawo_SolverDDM SHARED
  DYNSolverDDM.cpp DYNSubDomainDDM.cpp DYNSolverDDMFactory.cpp)
target_link_libraries(dynawo_SolverDDM
  dynawo_SolverCommon
  SuiteSparse::KLU
  LAPACK::LAPACK
  OpenMP::OpenMP_CXX)
```

---

## 3. Sub-Domain Object: `SubDomainDDM`

Each `SubDomainDDM` wraps one injector `SubModel` and owns its local Jacobian blocks and Schur complement:

```cpp
struct SubDomainDDM {
  int subModelIndex;          // index into ModelMulti::subModels_
  std::vector<int> intIdx;    // indices of internal vars x_i^int in global y
  std::vector<int> extIdx;    // indices of interface vars x_i^ext (bus voltage) in global y

  // Jacobian blocks (notation from reference method)
  Eigen::MatrixXd A1;   // ∂F_int/∂x_int  (dense, n_int × n_int)
  Eigen::MatrixXd A2;   // ∂F_int/∂x_ext  (dense, n_int × n_ext)
  Eigen::MatrixXd A3;   // ∂F_ext/∂x_int  (dense, n_ext × n_int)
  Eigen::MatrixXd A4;   // ∂F_ext/∂x_ext  (dense, n_ext × n_ext)

  Eigen::MatrixXd S;    // Schur complement: S_i = A4 - A3 * A1^{-1} * A2
  Eigen::VectorXd fInt; // local residual for internal vars
  Eigen::VectorXd fExt; // local residual for interface vars
  Eigen::VectorXd fTilde; // modified RHS: f~_i = fExt - A3 * A1^{-1} * fInt

  // LAPACK LU factorization storage
  std::vector<int> ipivA1, ipivS;
  bool factorized = false;

  // KLU alternative for large injectors
  klu_symbolic* kluSym = nullptr;
  klu_numeric*  kluNum = nullptr;
  klu_common    kluCommon;
  bool useKLU = false;   // set at init() if n_int > kluThreshold
};
```

### 3.1 Block Extraction

At each Jacobian evaluation, `SolverDDM` calls `model_->evalJt(t, cj_, smj)` to get the global sparse Jacobian, then extracts sub-domain blocks:

```cpp
void SolverDDM::evaluateAndPartitionJacobian() {
  SparseMatrix smj;
  smj.init(model_->sizeY(), model_->sizeY());
  model_->evalJt(tSolve_ + hCurrent_, cj_, smj);
  ++stats_.nje_;

  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(subDomains_.size()); ++i) {
    SubDomainDDM& sd = subDomains_[i];
    extractDenseBlock(smj, sd.intIdx, sd.intIdx, sd.A1);
    extractDenseBlock(smj, sd.intIdx, sd.extIdx, sd.A2);
    extractDenseBlock(smj, sd.extIdx, sd.intIdx, sd.A3);
    extractDenseBlock(smj, sd.extIdx, sd.extIdx, sd.A4);
  }
}
```

---

## 4. Schur-Complement Linear Solve

### 4.1 Per-Step Overview

At each Newton iteration, the global linear system `J Δy = −F` is solved via the Schur decomposition in three stages:

1. **Injector factorization** (parallel): factorize each `A1_i`, compute `S_i` and `f̃_i`
2. **Network reduced system** (serial KLU): assemble and solve the reduced interface system
3. **Back-substitution** (parallel): recover `Δx_int_i` for each injector

### 4.2 Dense LAPACK for Injectors

```cpp
void SubDomainDDM::factorizeAndComputeSchur() {
  int n    = static_cast<int>(A1.rows());
  int nrhs = static_cast<int>(A2.cols()) + 1;
  int info = 0;

  // In-place LU of A1
  ipivA1.resize(n);
  dgetrf_(&n, &n, A1.data(), &n, ipivA1.data(), &info);
  if (info != 0) throw DYNError(Error::SOLVER_ALGO, SolverDDMSingularA1, subModelIndex);

  // Solve A1 \ [A2 | fInt] in one DGETRS call
  Eigen::MatrixXd rhs(n, nrhs);
  rhs.leftCols(A2.cols()) = A2;
  rhs.col(A2.cols())      = fInt;
  int ldb = n;
  dgetrs_("N", &n, &nrhs, A1.data(), &n, ipivA1.data(),
          rhs.data(), &ldb, &info);

  // S_i = A4 - A3 * (A1^{-1} A2)
  S      = A4 - A3 * rhs.leftCols(A2.cols());
  // f̃_i = fExt - A3 * (A1^{-1} fInt)
  fTilde = fExt - A3 * rhs.col(A2.cols());

  // Factorize S_i for back-substitution
  int ns = static_cast<int>(S.rows());
  ipivS.resize(ns);
  dgetrf_(&ns, &ns, S.data(), &ns, ipivS.data(), &info);
  factorized = true;
}
```

### 4.3 Parallel Execution

```cpp
void SolverDDM::factorizeInjectorBlocks() {
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(subDomains_.size()); ++i)
    subDomains_[i].factorizeAndComputeSchur();
}
```

The star topology guarantees that sub-domains share **only** the network interface variables — there are no injector-to-injector couplings — so this loop is data-race free.

### 4.4 Reduced Network System and KLU Solve

The reduced interface system assembled from all Schur complements is:

\[
\left( D_4 + \sum_i S_i \right) \Delta V = -\left( f_{\text{net}} + \sum_i \tilde{f}_i \right)
\]

where \(D_4\) is the network's own `∂F_net/∂V` block extracted from `smj`.

```cpp
void SolverDDM::assembleAndSolveNetworkSystem() {
  // Start from network diagonal block
  smNet_ = networkBlock_;  // copy D4 extracted from smj

  // Add Schur contributions (scatter S_i into sparse network positions)
  for (const auto& sd : subDomains_)
    scatterAddDense(smNet_, sd.S, sd.extIdx);

  // KLU factorize (or refactorize if structure changed)
  if (factorizationForced_ || kluNetNum_ == nullptr) {
    if (kluNetNum_) { klu_free_numeric(&kluNetNum_, &kluCommon_); }
    if (kluNetSym_) { klu_free_symbolic(&kluNetSym_, &kluCommon_); }
    kluNetSym_ = klu_analyze(smNet_.nbRow(), smNet_.colPtr(),
                              smNet_.rowIndex(), &kluCommon_);
    kluNetNum_ = klu_factor(smNet_.colPtr(), smNet_.rowIndex(),
                             smNet_.values(), kluNetSym_, &kluCommon_);
  } else {
    klu_refactor(smNet_.colPtr(), smNet_.rowIndex(),
                 smNet_.values(), kluNetSym_, kluNetNum_, &kluCommon_);
  }

  // Assemble RHS: f_net + sum f̃_i
  std::fill(rhsNet_.begin(), rhsNet_.end(), 0.0);
  for (int k : networkVarIndices_)
    rhsNet_[k] = -vectorF_[k];
  for (const auto& sd : subDomains_)
    for (int j = 0; j < static_cast<int>(sd.extIdx.size()); ++j)
      rhsNet_[sd.extIdx[j]] -= sd.fTilde[j];

  klu_solve(kluNetSym_, kluNetNum_,
            static_cast<int>(rhsNet_.size()), 1,
            rhsNet_.data(), &kluCommon_);
  // rhsNet_ now holds ΔV
}
```

### 4.5 Back-Substitution

```cpp
void SolverDDM::backSubstituteAllInjectors() {
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(subDomains_.size()); ++i) {
    SubDomainDDM& sd = subDomains_[i];
    // Gather ΔV for this sub-domain's interface indices
    Eigen::VectorXd dV(sd.extIdx.size());
    for (int j = 0; j < static_cast<int>(sd.extIdx.size()); ++j)
      dV[j] = rhsNet_[sd.extIdx[j]];

    // Δx_int_i = A1^{-1} * (−fInt − A2 * ΔV)
    Eigen::VectorXd rhs_back = -sd.fInt - sd.A2 * dV;
    int n = static_cast<int>(rhs_back.size()), nrhs = 1, info = 0;
    dgetrs_("N", &n, &nrhs, sd.A1.data(), &n,
            sd.ipivA1.data(), rhs_back.data(), &n, &info);

    // Scatter corrections into global Δy
    for (int j = 0; j < static_cast<int>(sd.intIdx.size()); ++j)
      deltaY_[sd.intIdx[j]] = rhs_back[j];
    for (int j = 0; j < static_cast<int>(sd.extIdx.size()); ++j)
      deltaY_[sd.extIdx[j]] = dV[j];
  }
}
```

---

## 5. Time Integration: Self-Contained Variable-Step BDF-2

### 5.0 Feasibility and SUNDIALS Footprint

A SUNDIALS-free `SolverDDM` is **fully feasible**. The only caveat is that `Solver::Impl` allocates `SUNContext sundialsContext_`, `N_Vector sundialsVectorY_`, and `N_Vector sundialsVectorYp_` as base-class members inside `Solver::Impl::init()`. These are **passive memory handles** — they mirror the same storage as `vectorY_`/`vectorYp_` and carry zero algorithmic content. `SolverDDM` never calls any IDA, KINSOL, or SUNLinSol function; they are simply never touched after `init()`.

Every other component is implemented from scratch inside `SolverDDM`:
- **Time integration**: self-contained variable-step BDF-2 Newton loop (no IDA, no fixed-step Euler)
- **Inner linear solve**: Schur-complement decomposition with LAPACK `dgetrf`/`dgetrs` for injectors and raw KLU (SuiteSparse) for the network — no KINSOL, no `SolverKINEuler`
- **Algebraic restoration**: compact Newton loop with `cj=0` inside `SolverDDM::reinit()` — no `SolverKINAlgRestoration`
- **Root finding / zero-crossing**: `evalZMode` from `Solver::Impl` (pure C++, no SUNDIALS)

### 5.1 BDF-2 Formulation

For the global DAE `F(t, y, y') = 0` (assembled from injector and network residuals), the variable-step BDF-2 corrector defines `y'` at `t^(n+1)` as:

\[
y'^{(n+1)} = \alpha_0\, y^{(n+1)} + \alpha_1\, y^n + \beta_1\, y^{(n-1)}
\]

where the BDF-2 coefficients for variable step sizes \(h_n = t^{(n+1)} - t^n\) and \(h_{n-1} = t^n - t^{(n-1)}\) are:

\[
\theta = \frac{h_{n-1}}{h_n}, \quad
\alpha_0 = \frac{1 + 2\theta}{h_n(1+\theta)}, \quad
\alpha_1 = -\frac{1+\theta}{h_n}, \quad
\beta_1 = \frac{\theta^2}{h_n(1+\theta)}
\]

For the very first step (`h_{n-1}` unavailable), \(\beta_1 = 0\), \(\alpha_0 = 1/h_n\), \(\alpha_1 = -1/h_n\), which reduces to the BDF-1 (Backward-Euler) corrector. This is the standard single-step startup procedure for any BDF-2 integrator; from step 2 onward the full variable-step BDF-2 formula is used exclusively.

The residual `SolverDDM` drives to zero at each Newton iteration is:

\[
R(y^{(n+1)}) = F\!\left(t^{(n+1)},\; y^{(n+1)},\; \alpha_0\,y^{(n+1)} + \alpha_1\,y^n + \beta_1\,y^{(n-1)}\right)
\]

The Newton correction \(\Delta y\) satisfies \(J\,\Delta y = -R\) where:

\[
J = \frac{\partial F}{\partial y} + \alpha_0\,\frac{\partial F}{\partial y'}
\]

This \(c_j = \alpha_0\) is passed directly to Dynaωo's existing `model_->evalJt(t, cj, smj)` interface — the model layer already handles this coefficient correctly, as confirmed by how `SolverKINEuler::evalJ_KIN` passes `cj = 1/h` to `evalJt`. For `SolverDDM`, \(\alpha_0\) replaces the fixed `1/h` with the variable-step BDF-2 value.

### 5.2 Self-Contained Newton Loop

The Newton loop is implemented directly in `SolverDDM::solveStep()`, mirroring the control-flow structure of `SolverCommonFixedTimeStep::solveStepCommon()` but with every solver call replaced:

```cpp
void SolverDDM::solveStep(double tAim, double& tNxt) {
  int counter = 0;
  bool redoStep = false;

  saveContinuousVariables();  // snapshot y^n into ySave_ before attempting t^(n+1)

  do {
    if (counter >= maxNewtonTry_)
      throw DYNError(Error::SOLVER_ALGO, SolverDDMConvFail, maxNewtonTry_);

    // --- Compute BDF-2 coefficients for current hCurrent_ ---
    computeBDF2Coefficients();  // sets alpha0_, alpha1_, beta1_, cj_ = alpha0_

    // --- Newton iterations ---
    bool converged = false;
    int iter = 0;
    bool needJacUpdate = (stats_.nst_ == 0 || factorizationForced_);

    while (!converged && iter < mxiter_) {
      // 1. Update yp from BDF-2 formula
      computeYpFromBDF2();  // yp = alpha0*y + alpha1*yPrev_ + beta1*yPrevPrev_

      // 2. Evaluate global residual
      model_->copyContinuousVariables(vectorY_.data(), vectorYp_.data());
      model_->evalF(tSolve_ + hCurrent_, vectorY_.data(), vectorYp_.data(),
                    vectorF_.data());
      ++stats_.nre_;

      // 3. Check convergence (weighted WRMS on F)
      if (checkConvergence(iter)) { converged = true; break; }

      // 4. Evaluate/reuse Jacobian
      if (needJacUpdate || jacAge_ >= jacMaxAge_) {
        evaluateAndPartitionJacobian();   // evalJt(t, cj_, smj) then block-extract
        factorizeInjectorBlocks();        // parallel LAPACK or KLU
        assembleAndSolveNetworkSystem();  // assemble reduced system, KLU factorize
        jacAge_ = 0;
        needJacUpdate = false;
      }
      ++jacAge_;

      // 5. Schur solve: network correction then back-substitute injectors
      backSubstituteAllInjectors();  // parallel

      // 6. Apply Newton correction
      applyCorrection();
      ++stats_.nni_;
      ++iter;
    }

    if (!converged) {
      ++stats_.ncfn_;
      handleDivergence(redoStep);  // halve hCurrent_, restore ySave_, set factorizationForced_
      ++counter;
    } else {
      // 7. Evaluate zero-crossings and discrete variables
      SolverStatus_t status = CONV;
      model_->evalG(tSolve_ + hCurrent_, g1_);
      ++stats_.nge_;
      if (!std::equal(g0_.begin(), g0_.end(), g1_.begin())) {
        evalZMode(g0_, g1_, tSolve_ + hCurrent_);
        status = ROOT;
      }

      // 8. LTE-based step-size control (may reject step and set redoStep)
      adaptStepSize(status, redoStep);
    }
  } while (redoStep);

  updateTimeStep(tNxt);  // advance tSolve_, rotate history buffers
  ++stats_.nst_;
}
```

The control-flow skeleton mirrors `SolverCommonFixedTimeStep` for consistency, but every solver call is replaced:
- `maxNewtonTry_` outer retries on divergence before throwing a fatal error
- `factorizationForced_` flag to force Jacobian re-evaluation after a diverged or LTE-rejected step
- `saveContinuousVariables()` / `restoreContinuousVariables()` for retry on divergence
- `evalZMode` / root-stabilisation loop (pure C++ in `Solver::Impl`, no SUNDIALS)

### 5.3 Variable Step-Size Control

`SolverDDM` uses a genuine BDF-2 LTE estimate to drive step adaptation. After each accepted Newton convergence, the WRMS norm of the local truncation error is estimated over **differential variables only** (algebraic variables are excluded from LTE):

\[
\mathrm{LTE}_i = \frac{2h_n^2}{3(h_n + h_{n-1})} \left(y_i^{(n+1)} - 2y_i^n + y_i^{(n-1)}\right)
\]

\[
\|\mathrm{LTE}\|_\mathrm{wrms} = \sqrt{\frac{1}{N_\mathrm{diff}} \sum_i \left(\frac{\mathrm{LTE}_i}{r_\mathrm{tol}|y_i| + a_\mathrm{tol}}\right)^2}
\]

```cpp
void SolverDDM::adaptStepSize(SolverStatus_t& status, bool& redoStep) {
  if (status == ROOT) {
    // ROOT: always accept, reset to hMax_ for next step
    hNew_ = hMax_;
    redoStep = false;
    return;
  }

  double lteNorm = 0.0;
  int nDiff = 0;
  for (int i : differentialVariablesIndices_) {
    double w = 1.0 / (relTol_ * std::abs(vectorY_[i]) + absTol_);
    double d2 = vectorY_[i] - 2.0 * yPrev_[i] + yPrevPrev_[i];
    double lte = (2.0 * hCurrent_ * hCurrent_) / (3.0 * (hCurrent_ + hPrev_)) * d2;
    lteNorm += (lte * w) * (lte * w);
    ++nDiff;
  }
  if (nDiff == 0) { redoStep = false; return; }  // pure algebraic system
  lteNorm = std::sqrt(lteNorm / nDiff);

  // Step-size proposal based on LTE
  const double safety = 0.9;
  double factor = safety * std::pow(1.0 / std::max(lteNorm, 1e-10), 1.0 / 3.0);
  // kReduceStep_ is used as a clamp bound on the factor, not as the factor itself
  factor = std::min(1.0 / kReduceStep_, std::max(kReduceStep_, factor));
  hNew_ = std::min(hMax_, std::max(hMin_, hCurrent_ * factor));

  if (lteNorm > 1.0) {
    // LTE too large: reject step, restore, retry with hNew_
    restoreContinuousVariables();
    factorizationForced_ = true;
    redoStep = true;
    ++stats_.netf_;
  } else {
    redoStep = false;
  }
}
```

The growth and shrink bounds clamp the LTE-derived factor using `kReduceStep_` as a bound — reusing a familiar parameter name, but here it is a **limit on the factor**, not a fixed multiplier. The actual step proposal always comes from the LTE norm, making this a genuine variable-step controller. `hMin_` and `hMax_` are mandatory parameters that set hard limits.

**On a ROOT event**, the step is always accepted regardless of LTE (the root is the dominant event), and `hNew_` is set to `hMax_` to allow the solver to stretch out again after the disturbance.

### 5.4 Algebraic Restoration Without KINSOL

`reinit()` after a mode change requires solving for the new algebraic variable values with differential variables held fixed. `SolverDDM` replaces the KINSOL-based `SolverKINAlgRestoration` with a compact Newton loop using the same Schur decomposition but with `cj = 0` (pure algebraic Jacobian):

```cpp
void SolverDDM::reinit() {
  modeChangeType_t modeChange = model_->getModeChangeType();
  if (modeChange < minimumModeChangeTypeForAlgebraicRestoration_) return;

  int counter = 0;
  do {
    model_->rotateBuffers();
    state_.reset();

    // Zero out yp for algebraic variables
    const auto& yType = model_->getYType();
    for (int i = 0; i < model_->sizeY(); ++i)
      if (yType[i] != DYN::DIFFERENTIAL) vectorYp_[i] = 0.0;

    // Newton loop with cj=0: drives only algebraic residuals to zero
    bool convAlg = runAlgebraicNewtonLoop(/*forceJacUpdate=*/
                                          modeChange == ALGEBRAIC_J_UPDATE_MODE);
    if (!convAlg)
      throw DYNError(Error::SOLVER_ALGO, SolverDDMAlgRestFail);

    model_->reinitMode();

    // Flush BDF-2 history: restart with BDF-1 for next time step
    bdf1Bootstrap_ = true;
    factorizationForced_ = true;

    // Root stabilization
    model_->evalG(tSolve_, g1_);
    ++stats_.nge_;
    if (std::equal(g0_.begin(), g0_.end(), g1_.begin())) break;
    g0_.assign(g1_.begin(), g1_.end());
    evalZMode(g0_, g1_, tSolve_);
    modeChange = model_->getModeChangeType();
    if (++counter >= maxNumberUnstableRoots)
      throw DYNError(Error::SOLVER_ALGO, SolverDDMUnstableRoots);
  } while (modeChange >= minimumModeChangeTypeForAlgebraicRestoration_);
}

bool SolverDDM::runAlgebraicNewtonLoop(bool forceJacUpdate) {
  const double cj_alg = 0.0;
  for (int iter = 0; iter < mxiterAlg_; ++iter) {
    model_->copyContinuousVariables(vectorY_.data(), vectorYp_.data());
    model_->evalF(tSolve_, vectorY_.data(), vectorYp_.data(), vectorF_.data());
    if (checkAlgConvergence()) return true;

    if (iter == 0 || forceJacUpdate) {
      // Evaluate Jacobian with cj=0 (algebraic only)
      SparseMatrix smj;
      smj.init(model_->sizeY(), model_->sizeY());
      model_->evalJt(tSolve_, cj_alg, smj);
      partitionJacobianBlocks(smj);
      factorizeInjectorBlocks();
      assembleAndSolveNetworkSystem();
      forceJacUpdate = false;
    }
    backSubstituteAllInjectors();
    applyCorrection();
  }
  return false;
}
```

Flushing `bdf1Bootstrap_ = true` on every `reinit()` ensures BDF-2 restarts cleanly with one BDF-1 step, avoiding use of a stale `y^{(n-1)}` from before the event.

### 5.5 History Buffer Management

The solver maintains additional state beyond `vectorY_`/`vectorYp_` from `Solver::Impl`:

```cpp
// Additional members in SolverDDM (summary)
std::vector<double> yPrev_;       // y^n   (rotated on each accepted step)
std::vector<double> yPrevPrev_;   // y^(n-1)
std::vector<double> ySave_;       // snapshot before Newton attempts (for retry)
double hPrev_;                     // h_{n-1}: step size of the previous accepted step
bool   bdf1Bootstrap_;             // true: use BDF-1 this step (first step or post-reinit)
```

The `ySave_` buffer is used to restore state on Newton divergence or LTE rejection. The `yPrev_` and `yPrevPrev_` buffers are rotated on each **accepted** step only — they are never updated when a step is rejected.

---

## 6. Initialization: `calculateIC`

`calculateIC` follows the same algebraic-loop-stabilization pattern as `SolverCommonFixedTimeStep::calculateICCommon()`:

1. Evaluate `g0_` (initial zero-crossings)
2. `evalZMode` to propagate discrete variables
3. Run `runAlgebraicNewtonLoop(true)` with `cj=0` to find consistent algebraic initial conditions
4. Re-evaluate `g1_`, check root stabilization; repeat if roots changed
5. Set `bdf1Bootstrap_ = true` so the first time step uses BDF-1

No KINSOL, no IDA `IDACalcIC` — the same Newton loop as `reinit()` handles consistency.

---

## 7. Interface Variable Detection at `init()`

At `init()` time, `SolverDDM` traverses `ModelMulti`'s sub-model list and connector graph to populate `subDomains_` and `networkVarIndices_`:

```cpp
void SolverDDM::init(const std::shared_ptr<Model>& model, double t0, double tEnd) {
  tSolve_ = t0;
  tEnd_   = tEnd;
  hCurrent_ = hMax_;
  hNew_     = hMax_;
  bdf1Bootstrap_ = true;
  jacAge_ = 0;
  factorizationForced_ = true;

  Solver::Impl::init(t0, model);
  Solver::Impl::resetStats();

  vectorF_.resize(model_->sizeF());
  yPrev_.resize(model_->sizeY(), 0.0);
  yPrevPrev_.resize(model_->sizeY(), 0.0);
  ySave_.resize(model_->sizeY(), 0.0);
  deltaY_.resize(model_->sizeY(), 0.0);
  rhsNet_.resize(model_->sizeY(), 0.0);

  g0_.assign(model_->sizeG(), ROOT_DOWN);
  g1_.assign(model_->sizeG(), ROOT_DOWN);

  setDifferentialVariablesIndices();

  // Build sub-domain map from ModelMulti connector topology
  buildSubDomainMap();  // populates subDomains_, networkVarIndices_
}
```

`buildSubDomainMap()` identifies injector sub-models (those with bus-voltage interface connections) by inspecting `model_->getSubModels()` and the connector variable types. Network variables are those belonging to `ModelNetwork`/`ModelBus` sub-models.

---

## 8. Parameters

| Parameter | Type | Mandatory | Description |
|---|---|---|---|
| `hMin` | double | yes | Minimum time step |
| `hMax` | double | yes | Maximum / initial time step |
| `kReduceStep` | double | yes | Bounds clamp on LTE step-size factor |
| `maxNewtonTry` | int | yes | Max outer retries before fatal error |
| `mxiter` | int | no | Max Newton iterations per step (default 15) |
| `relTol` | double | no | Relative tolerance for LTE control (default 1e-4) |
| `absTol` | double | no | Absolute tolerance for LTE control (default 1e-4) |
| `jacMaxAge` | int | no | Max Newton iterations before forced Jacobian update (default 5) |
| `linearSolverInjector` | string | no | `"dense"` (default) or `"sparse"` for injector A1 blocks |
| `kluThreshold` | int | no | Min injector size to switch to KLU (default 500) |
| `numThreads` | int | no | OpenMP thread count (default: system `OMP_NUM_THREADS`) |

---

## 9. Implementation Phasing

### Phase 1: Sequential Schur baseline (BDF-2 variable step, serial)
- Implement `SolverDDM` with all Schur machinery but `#pragma omp` directives disabled
- Validate numerically against `SolverIDA` on IEEE 39-bus and Nordic test networks
- Confirm BDF-2 LTE step control matches IDA trajectories within tolerance

### Phase 2: OpenMP parallelism over injectors
- Enable `#pragma omp parallel for` in `factorizeInjectorBlocks()` and `backSubstituteAllInjectors()`
- Measure strong scaling (2, 4, 8, 16 threads) on the Nordic test network
- Ensure thread-safety of `SubDomainDDM` objects (each thread owns its `SubDomainDDM` exclusively)

### Phase 3: Jacobian aging and reuse
- Implement `jacAge_` / `jacMaxAge_` to skip Jacobian re-evaluation across Newton iterations
- Tune `jacMaxAge` for the BDF-2 modified Newton strategy (typically 3–5 steps)

### Phase 4: EP Early-Stopping
- Add per-sub-domain convergence flag: once `||ΔV_i||` falls below threshold, freeze that sub-domain's contributions to the reduced RHS
- Implement asynchronous Schur update: skip `S_i` recomputation for converged sub-domains
- Validate that EP variant matches Phase 1 baseline numerically

---

## 10. Key Design Decisions

| Decision | Rationale |
|---|---|
| Dense Eigen + LAPACK for injectors | Individual injectors have 10–200 variables; `dgetrf` outperforms KLU below ~500 vars |
| KLU for network system | Already in SuiteSparse dependency stack; reuses `klu_analyze`/`klu_factor` pattern from `SolverIDA` |
| No KINSOL anywhere | Eliminates the KINSOL Newton wrapper overhead; hand-rolled loop gives direct control over convergence criteria and Jacobian reuse |
| BDF-2 only (BDF-1 as startup only) | Matches the order used in the reference method; avoids the variable-order complexity of full IDA BDF1–5 |
| History flush on `reinit()` | Prevents stale `y^{(n-1)}` from polluting the BDF-2 derivative estimate after a topology change |
| `kReduceStep_` as LTE clamp | Familiar parameter name from existing solvers; semantics change from "fixed multiplier" to "factor bound" — document clearly in PAR file comments |
