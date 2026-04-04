# SolverDDM — Single-Level Schur-Complement Domain Decomposition Solver for Dynawo

## Executive Summary

This document is a design brainstorm for `SolverDDM`, a new Dynawo solver that implements the
**single-level Schur-complement domain decomposition method** (network vs. injectors) as described
in the reference papers. The solver targets large-scale time-domain dynamic simulation with
variable-step BDF-2 integration, OpenMP parallelism over injector sub-domains, and zero dependency
on SUNDIALS IDA or KINSOL at runtime. It slots into `dynawo/sources/Solvers/` as a peer of
`SolverIDA` and `SolverSIM`.

---

## 1. Background and Reference Method

The decomposition method implemented in `SolverDDM` is the **single-level Schur-complement DDM**
described in the journal paper and Chapter 4 of the corresponding thesis. The algorithm performs a
**topological star-shaped decomposition**: each injector (synchronous machine, load, motor, wind
turbine, etc.) is one sub-domain, and the entire electric network is the central sub-domain.
**Only this single-level decomposition is in scope** — the two-level extension (Chapter 5 of the
thesis) that further decomposes the network itself is explicitly excluded.

The interface variables are fixed by physics:

- **Injector → network**: injected current at the connection bus (`x_ext_i`)
- **Network → injector**: terminal voltage of that bus (`V_ext`)

No METIS/Scotch partitioning is needed; the decomposition is read directly from the `DYNConnector`
topology at `init()` time.

The reference method defines two parallel variants on top of the single-level decomposition.
Both share the same injector-vs-network structure:

| Variant | Injector update strategy | Network solve | Parallelism |
|---------|--------------------------|---------------|-------------|
| **(P)** Basic parallel | All injectors solved each iteration; global matrix update criterion | One KLU solve on reduced system | OpenMP over injectors |
| **(EP)** Enhanced parallel | Per-injector local convergence: converged sub-domains stop early; asynchronous matrix updates | One KLU solve on reduced system | OpenMP over injectors |

Variant (EP) provides additional **numerical acceleration** on top of the computational parallelism:
injectors that converge early are skipped in subsequent iterations, reducing total work per time
step. Both variants remain single-level and are in scope. The recommended implementation order is
(P) first as a correctness baseline, then (EP) as the production target.

---

## 2. Solver Architecture and Location

### 2.1 Directory Layout

```text
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

`SolverDDM` inherits from `Solver::Impl` exactly as `SolverIDA` and `SolverCommonFixedTimeStep` do.
It overrides the mandatory virtual interface:

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
  std::vector<double> yPrev_;        // y[n]
  std::vector<double> yPrevPrev_;    // y[n-1]
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
  std::vector<SubDomainDDM> subDomains_;   // one per injector SubModel
  std::vector<int>          networkVarIndices_; // indices of network vars in global y

  // Jacobian management
  int  jacAge_;
  int  jacMaxAge_;
  bool factorizationForced_;

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

Each `SubDomainDDM` wraps one injector `SubModel` and owns its local Jacobian blocks and Schur
complement:

```cpp
struct SubDomainDDM {
  int subModelIndex;          // index into ModelMulti::subModels_
  std::vector<int> intIdx;    // indices of internal vars x_int in global y
  std::vector<int> extIdx;    // indices of interface vars x_ext (bus voltage) in global y

  // Jacobian blocks (notation follows the reference method)
  // A1 = dF_int/dx_int  (dense, n_int x n_int)
  // A2 = dF_int/dx_ext  (dense, n_int x n_ext)
  // A3 = dF_ext/dx_int  (dense, n_ext x n_int)
  // A4 = dF_ext/dx_ext  (dense, n_ext x n_ext)
  Eigen::MatrixXd A1, A2, A3, A4;

  // Schur complement: S_i = A4 - A3 * A1^{-1} * A2
  Eigen::MatrixXd S;

  Eigen::VectorXd fInt;    // local residual for internal vars
  Eigen::VectorXd fExt;    // local residual for interface vars
  // Modified RHS: fTilde_i = fExt - A3 * A1^{-1} * fInt
  Eigen::VectorXd fTilde;

  // LAPACK LU factorisation storage
  std::vector<int> ipivA1, ipivS;
  bool factorized = false;

  // KLU fallback for large injectors
  klu_symbolic* kluSym  = nullptr;
  klu_numeric*  kluNum  = nullptr;
  klu_common    kluCommon;
  bool useKLU = false;   // set at init() when n_int > kluThreshold
};
```

### 3.1 Block Extraction

At each Jacobian evaluation, `SolverDDM` calls `model_->evalJt(t, cj_, smj)` to obtain the global
sparse Jacobian, then extracts sub-domain blocks in parallel:

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

At each Newton iteration the global system `J * dy = -F` is solved in three stages:

1. **Injector factorisation** (parallel): factorise each `A1_i`, compute `S_i` and `fTilde_i`
2. **Network reduced system** (serial KLU): assemble and solve the reduced interface system
3. **Back-substitution** (parallel): recover `dx_int_i` for each injector

### 4.2 Dense LAPACK for Injectors

```cpp
void SubDomainDDM::factorizeAndComputeSchur() {
  int n    = static_cast<int>(A1.rows());
  int nrhs = static_cast<int>(A2.cols()) + 1;
  int info = 0;

  // In-place LU factorisation of A1
  ipivA1.resize(n);
  dgetrf_(&n, &n, A1.data(), &n, ipivA1.data(), &info);
  if (info != 0)
    throw DYNError(Error::SOLVER_ALGO, SolverDDMSingularA1, subModelIndex);

  // Solve A1 \ [A2 | fInt] in a single DGETRS call
  Eigen::MatrixXd rhs(n, nrhs);
  rhs.leftCols(A2.cols()) = A2;
  rhs.col(A2.cols())      = fInt;
  int ldb = n;
  dgetrs_("N", &n, &nrhs, A1.data(), &n, ipivA1.data(),
          rhs.data(), &ldb, &info);

  // S_i = A4 - A3 * (A1^{-1} * A2)
  S      = A4 - A3 * rhs.leftCols(A2.cols());
  // fTilde_i = fExt - A3 * (A1^{-1} * fInt)
  fTilde = fExt - A3 * rhs.col(A2.cols());

  // Factorise S_i for later back-substitution
  int ns = static_cast<int>(S.rows());
  ipivS.resize(ns);
  dgetrf_(&ns, &ns, S.data(), &ns, ipivS.data(), &info);
  factorized = true;
}
```

### 4.3 Parallel Injector Factorisation

```cpp
void SolverDDM::factorizeInjectorBlocks() {
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(subDomains_.size()); ++i)
    subDomains_[i].factorizeAndComputeSchur();
}
```

The star topology guarantees that sub-domains share **only** the network interface variables —
there are no injector-to-injector couplings — so this loop is data-race free.

### 4.4 Reduced Network System and KLU Solve

The reduced interface system assembled from all Schur complements is:

$$
\bigl(D_4 + \textstyle\sum_i S_i\bigr)\,\Delta V
= -\bigl(f_{\text{net}} + \textstyle\sum_i \tilde{f}_i\bigr)
$$

where $D_4$ is the network's own `dF_net/dV` block extracted from `smj`.

```cpp
void SolverDDM::assembleAndSolveNetworkSystem() {
  smNet_ = networkBlock_;   // copy D4 extracted from smj

  // Scatter each sub-domain's Schur complement into the network sparse matrix
  for (const auto& sd : subDomains_)
    scatterAddDense(smNet_, sd.S, sd.extIdx);

  // KLU: full factorisation when forced, numeric refactorisation otherwise
  if (factorizationForced_ || kluNetNum_ == nullptr) {
    if (kluNetNum_) klu_free_numeric(&kluNetNum_, &kluCommon_);
    if (kluNetSym_) klu_free_symbolic(&kluNetSym_, &kluCommon_);
    kluNetSym_ = klu_analyze(smNet_.nbRow(), smNet_.colPtr(),
                             smNet_.rowIndex(), &kluCommon_);
    kluNetNum_ = klu_factor(smNet_.colPtr(), smNet_.rowIndex(),
                            smNet_.values(), kluNetSym_, &kluCommon_);
  } else {
    klu_refactor(smNet_.colPtr(), smNet_.rowIndex(),
                 smNet_.values(), kluNetSym_, kluNetNum_, &kluCommon_);
  }

  // Assemble RHS = -(f_net + sum fTilde_i)
  std::fill(rhsNet_.begin(), rhsNet_.end(), 0.0);
  for (int k : networkVarIndices_)
    rhsNet_[k] = -vectorF_[k];
  for (const auto& sd : subDomains_)
    for (int j = 0; j < static_cast<int>(sd.extIdx.size()); ++j)
      rhsNet_[sd.extIdx[j]] -= sd.fTilde[j];

  klu_solve(kluNetSym_, kluNetNum_,
            static_cast<int>(rhsNet_.size()), 1,
            rhsNet_.data(), &kluCommon_);
  // rhsNet_ now contains dV
}
```

### 4.5 Back-Substitution

```cpp
void SolverDDM::backSubstituteAllInjectors() {
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(subDomains_.size()); ++i) {
    SubDomainDDM& sd = subDomains_[i];

    // Gather dV for this sub-domain's interface indices
    Eigen::VectorXd dV(sd.extIdx.size());
    for (int j = 0; j < static_cast<int>(sd.extIdx.size()); ++j)
      dV[j] = rhsNet_[sd.extIdx[j]];

    // dx_int_i = A1^{-1} * (-fInt - A2 * dV)
    Eigen::VectorXd rhs_back = -sd.fInt - sd.A2 * dV;
    int n = static_cast<int>(rhs_back.size()), nrhs = 1, info = 0;
    dgetrs_("N", &n, &nrhs, sd.A1.data(), &n,
            sd.ipivA1.data(), rhs_back.data(), &n, &info);

    // Scatter corrections into global dy
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

A SUNDIALS-free `SolverDDM` is **fully feasible**. The only caveat is that `Solver::Impl` allocates
`SUNContext sundialsContext_`, `N_Vector sundialsVectorY_`, and `N_Vector sundialsVectorYp_` as
base-class members inside `Solver::Impl::init()`. These are **passive memory handles** — they
mirror the same storage as `vectorY_`/`vectorYp_` and carry zero algorithmic content. `SolverDDM`
never calls any IDA, KINSOL, or SUNLinSol function; the handles are simply never touched after
`init()`.

Every other component is implemented from scratch inside `SolverDDM`:

| Concern | Implementation |
|---------|----------------|
| Time integration | Self-contained variable-step BDF-2 Newton loop — no IDA, no fixed-step Euler |
| Inner linear solve | Schur decomposition: LAPACK `dgetrf`/`dgetrs` for injectors, raw KLU for network — no KINSOL, no `SolverKINEuler` |
| Algebraic restoration | Compact Newton loop with `cj=0` in `SolverDDM::reinit()` — no `SolverKINAlgRestoration` |
| Root finding | `evalZMode` from `Solver::Impl` (pure C++, no SUNDIALS) |

### 5.1 BDF-2 Formulation

For the global DAE $F(t,\,y,\,y') = 0$ the variable-step BDF-2 corrector approximates $y'$ at
$t^{n+1}$ as:

$$
y'^{\,n+1} = \alpha_0\,y^{n+1} + \alpha_1\,y^{n} + \beta_1\,y^{n-1}
$$

where, with $h_n = t^{n+1}-t^n$ and $\theta = h_{n-1}/h_n$:

$$
\alpha_0 = \frac{1+2\theta}{h_n(1+\theta)}, \qquad
\alpha_1 = -\frac{1+\theta}{h_n}, \qquad
\beta_1  = \frac{\theta^2}{h_n(1+\theta)}
$$

> **First-step bootstrap:** when $h_{n-1}$ is unavailable ($\theta = 0$), the formula reduces to
> BDF-1 (Backward Euler): $\alpha_0 = 1/h_n$, $\alpha_1 = -1/h_n$, $\beta_1 = 0$.
> From step 2 onward the full variable-step BDF-2 formula is used exclusively.

The residual driven to zero at each Newton iteration is:

$$
R\bigl(y^{n+1}\bigr) =
F\!\left(t^{n+1},\;y^{n+1},\;
\alpha_0\,y^{n+1}+\alpha_1\,y^{n}+\beta_1\,y^{n-1}\right)
$$

The Newton correction $\Delta y$ satisfies $J\,\Delta y = -R$ with:

$$
J = \frac{\partial F}{\partial y} + \alpha_0\,\frac{\partial F}{\partial y'}
$$

The scalar $c_j = \alpha_0$ is passed directly to `model_->evalJt(t, cj, smj)`, which already
accepts this coefficient (replacing the fixed `1/h` used by `SolverKINEuler`).

### 5.2 Self-Contained Newton Loop

```cpp
void SolverDDM::solveStep(double tAim, double& tNxt) {
  int  counter  = 0;
  bool redoStep = false;

  saveContinuousVariables();   // snapshot y[n] into ySave_ before attempting t[n+1]

  do {
    if (counter >= maxNewtonTry_)
      throw DYNError(Error::SOLVER_ALGO, SolverDDMConvFail, maxNewtonTry_);

    computeBDF2Coefficients();   // sets alpha0_, alpha1_, beta1_, cj_ = alpha0_

    bool converged     = false;
    int  iter          = 0;
    bool needJacUpdate = (stats_.nst_ == 0 || factorizationForced_);

    while (!converged && iter < mxiter_) {
      computeYpFromBDF2();   // yp = alpha0*y + alpha1*yPrev_ + beta1*yPrevPrev_

      model_->copyContinuousVariables(vectorY_.data(), vectorYp_.data());
      model_->evalF(tSolve_ + hCurrent_,
                    vectorY_.data(), vectorYp_.data(), vectorF_.data());
      ++stats_.nre_;

      if (checkConvergence(iter)) { converged = true; break; }

      if (needJacUpdate || jacAge_ >= jacMaxAge_) {
        evaluateAndPartitionJacobian();  // evalJt(t, cj_, smj) then block-extract
        factorizeInjectorBlocks();       // parallel LAPACK / KLU per sub-domain
        assembleAndSolveNetworkSystem(); // assemble reduced system, KLU factorise
        jacAge_       = 0;
        needJacUpdate = false;
      }
      ++jacAge_;

      backSubstituteAllInjectors();  // parallel
      applyCorrection();
      ++stats_.nni_;
      ++iter;
    }

    if (!converged) {
      ++stats_.ncfn_;
      handleDivergence(redoStep);   // halve hCurrent_, restore ySave_
      ++counter;
    } else {
      SolverStatus_t status = CONV;
      model_->evalG(tSolve_ + hCurrent_, g1_);
      ++stats_.nge_;
      if (!std::equal(g0_.begin(), g0_.end(), g1_.begin())) {
        evalZMode(g0_, g1_, tSolve_ + hCurrent_);
        status = ROOT;
      }
      adaptStepSize(status, redoStep);  // LTE-based; may reject and set redoStep
    }
  } while (redoStep);

  updateTimeStep(tNxt);   // advance tSolve_, rotate history buffers
  ++stats_.nst_;
}
```

Control-flow notes (mirrors `SolverCommonFixedTimeStep`, all solver calls replaced):

- `maxNewtonTry_` — outer retry limit before a fatal error is thrown
- `factorizationForced_` — forces Jacobian re-evaluation after divergence or LTE rejection
- `saveContinuousVariables()` / `restoreContinuousVariables()` — snapshot/restore on retry
- `evalZMode` — pure C++ in `Solver::Impl`, no SUNDIALS involvement

### 5.3 Variable Step-Size Control

After each accepted Newton convergence, the WRMS norm of the BDF-2 local truncation error is
estimated over **differential variables only**:

$$
\text{LTE}_i = \frac{2h_n^2}{3(h_n+h_{n-1})}
\bigl(y_i^{n+1} - 2y_i^{n} + y_i^{n-1}\bigr)
$$

$$
\|\text{LTE}\|_{\text{wrms}} =
\sqrt{\frac{1}{N_{\text{diff}}}
\sum_i \left(\frac{\text{LTE}_i}{r_{\text{tol}}|y_i|+a_{\text{tol}}}\right)^{\!2}}
$$

```cpp
void SolverDDM::adaptStepSize(SolverStatus_t& status, bool& redoStep) {
  if (status == ROOT) {
    hNew_     = hMax_;   // stretch out after disturbance
    redoStep  = false;
    return;
  }

  double lteNorm = 0.0;
  int    nDiff   = 0;
  for (int i : differentialVariablesIndices_) {
    double w   = 1.0 / (relTol_ * std::abs(vectorY_[i]) + absTol_);
    double d2  = vectorY_[i] - 2.0*yPrev_[i] + yPrevPrev_[i];
    double lte = (2.0*hCurrent_*hCurrent_) / (3.0*(hCurrent_+hPrev_)) * d2;
    lteNorm   += (lte*w) * (lte*w);
    ++nDiff;
  }
  if (nDiff == 0) { redoStep = false; return; }   // pure algebraic system
  lteNorm = std::sqrt(lteNorm / nDiff);

  // LTE-derived factor; kReduceStep_ is a clamp bound, not a fixed multiplier
  const double safety = 0.9;
  double factor = safety * std::pow(1.0 / std::max(lteNorm, 1e-10), 1.0/3.0);
  factor = std::min(1.0/kReduceStep_, std::max(kReduceStep_, factor));
  hNew_  = std::min(hMax_, std::max(hMin_, hCurrent_ * factor));

  if (lteNorm > 1.0) {
    restoreContinuousVariables();
    factorizationForced_ = true;
    redoStep = true;
    ++stats_.netf_;
  } else {
    redoStep = false;
  }
}
```

> **`kReduceStep_` semantics:** unlike in the fixed-step solvers where it is a fixed multiplier,
> here it acts as an upper/lower **bound on the LTE-derived factor**. The actual step proposal
> always comes from the LTE norm.

### 5.4 Algebraic Restoration Without KINSOL

`reinit()` solves for consistent algebraic variables after a mode change using the same Schur
decomposition with `cj = 0` (pure algebraic Jacobian) — no `SolverKINAlgRestoration`:

```cpp
void SolverDDM::reinit() {
  modeChangeType_t modeChange = model_->getModeChangeType();
  if (modeChange < minimumModeChangeTypeForAlgebraicRestoration_) return;

  int counter = 0;
  do {
    model_->rotateBuffers();
    state_.reset();

    // Zero yp for all algebraic variables
    const auto& yType = model_->getYType();
    for (int i = 0; i < model_->sizeY(); ++i)
      if (yType[i] != DYN::DIFFERENTIAL) vectorYp_[i] = 0.0;

    bool convAlg = runAlgebraicNewtonLoop(
                     modeChange == ALGEBRAIC_J_UPDATE_MODE);
    if (!convAlg)
      throw DYNError(Error::SOLVER_ALGO, SolverDDMAlgRestFail);

    model_->reinitMode();

    // Flush BDF-2 history: next step restarts with BDF-1
    bdf1Bootstrap_       = true;
    factorizationForced_ = true;

    // Root stabilisation
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
```

Setting `bdf1Bootstrap_ = true` on every `reinit()` ensures BDF-2 restarts cleanly from one
BDF-1 step, preventing stale `y[n-1]` values from before the event from corrupting the derivative
estimate.

#### `runAlgebraicNewtonLoop` with F-Scaling

The algebraic Newton loop borrows the **F-scaling** technique from `SolverKINAlgRestoration`
(see `DYNSolverKINAlgRestoration.cpp`, `solveStrategy()`): the initial residual is evaluated
once before the Newton iterations and used to build a diagonal scale vector `fScale`. This
improves the WRMS convergence check for residuals with large absolute magnitudes (e.g. power
injections in per-unit systems where `|F_i| >> 1` on the first call). The implementation is
~10 lines, adds no SUNDIALS dependency, and is entirely compatible with the Schur linear solve.

```cpp
bool SolverDDM::runAlgebraicNewtonLoop(bool forceJacUpdate) {
  const double cj_alg = 0.0;

  // --- F-scaling: evaluate F once to build a diagonal scale vector -----------
  // Mirrors SolverKINAlgRestoration::solveStrategy() — no SUNDIALS needed.
  model_->copyContinuousVariables(vectorY_.data(), vectorYp_.data());
  model_->evalF(tSolve_, vectorY_.data(), vectorYp_.data(), vectorF_.data());

  std::vector<double> fScale(model_->sizeF(), 1.0);
  for (int i = 0; i < model_->sizeF(); ++i)
    if (std::abs(vectorF_[i]) > 1.0)
      fScale[i] = 1.0 / std::abs(vectorF_[i]);
  // ---------------------------------------------------------------------------

  for (int iter = 0; iter < mxiterAlg_; ++iter) {
    // Re-evaluate F (skipped on iter==0: already evaluated above for scaling)
    if (iter > 0) {
      model_->copyContinuousVariables(vectorY_.data(), vectorYp_.data());
      model_->evalF(tSolve_, vectorY_.data(), vectorYp_.data(), vectorF_.data());
    }

    if (checkAlgConvergence(fScale)) return true;   // weighted WRMS norm < fnormtolAlg_

    if (iter == 0 || forceJacUpdate) {
      SparseMatrix smj;
      smj.init(model_->sizeY(), model_->sizeY());
      model_->evalJt(tSolve_, cj_alg, smj);   // cj=0: algebraic Jacobian only
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

The `checkAlgConvergence(fScale)` helper computes:

$$
\|R\|_{\text{wrms}} = \sqrt{\frac{1}{N} \sum_i \bigl(F_i \cdot \text{fScale}_i\bigr)^2}
$$

and returns `true` when this norm falls below `fnormtolAlg_`. This is strictly the same criterion
as KINSOL's `KIN_NONE` strategy, so numerical behaviour is directly comparable to
`SolverKINAlgRestoration`.

> **Why not use `SolverKINAlgRestoration` directly?**  
> `SolverKINAlgRestoration` calls `KINSolve`, `N_VMake_Serial`, and `SUNLinSol_KLU` internally —
> it has no hook to inject the Schur linear solve. The variable-filtering in `initVarAndEqTypes()`
> (algebraic vs. differential classification) is also incompatible with the Schur partition
> (injector-internal vs. interface). The F-scaling logic is the only numerically valuable piece;
> it is cleanly extracted here without any KINSOL dependency.

### 5.5 History Buffer Management

```cpp
// Additional state in SolverDDM beyond Solver::Impl
std::vector<double> yPrev_;      // y[n]   — rotated on each *accepted* step
std::vector<double> yPrevPrev_;  // y[n-1] — rotated on each *accepted* step
std::vector<double> ySave_;      // snapshot before Newton attempts (retry/LTE restore)
double hPrev_;                    // h_{n-1}: step size of previous accepted step
bool   bdf1Bootstrap_;            // true => use BDF-1 this step
```

`yPrev_` and `yPrevPrev_` are **never** updated on a rejected step (Newton divergence or LTE
failure). Only `ySave_` is used for restore on those paths.

---

## 6. Initialization: `calculateIC`

`calculateIC` follows the same algebraic-stabilisation pattern as
`SolverCommonFixedTimeStep::calculateICCommon()`:

1. Evaluate `g0_` (initial zero-crossings)
2. `evalZMode` to propagate discrete variables
3. `runAlgebraicNewtonLoop(true)` with `cj = 0` — consistent algebraic initial conditions
4. Re-evaluate `g1_`; repeat if roots changed
5. Set `bdf1Bootstrap_ = true` so the first time step uses BDF-1

No KINSOL, no `IDACalcIC` — the same internal Newton loop as `reinit()` handles consistency.

---

## 7. Interface Variable Detection at `init()`

```cpp
void SolverDDM::init(const std::shared_ptr<Model>& model, double t0, double tEnd) {
  tSolve_  = t0;  tEnd_ = tEnd;
  hCurrent_ = hNew_ = hMax_;
  bdf1Bootstrap_    = true;
  jacAge_           = 0;
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
  buildSubDomainMap();   // populates subDomains_, networkVarIndices_
}
```

`buildSubDomainMap()` identifies injector sub-models (those with bus-voltage interface connections)
by inspecting `model_->getSubModels()` and the connector variable types. Network variables are
those belonging to `ModelNetwork`/`ModelBus` sub-models.

---

## 8. Parameters

| Parameter | Type | Mandatory | Default | Description |
|-----------|------|-----------|---------|-------------|
| `hMin` | double | yes | — | Minimum time step |
| `hMax` | double | yes | — | Maximum / initial time step |
| `kReduceStep` | double | yes | — | Bounds clamp on LTE step-size factor |
| `maxNewtonTry` | int | yes | — | Max outer retries before fatal error |
| `mxiter` | int | no | 15 | Max Newton iterations per step |
| `relTol` | double | no | 1e-4 | Relative tolerance for LTE control |
| `absTol` | double | no | 1e-4 | Absolute tolerance for LTE control |
| `jacMaxAge` | int | no | 5 | Newton iterations before forced Jacobian update |
| `linearSolverInjector` | string | no | `dense` | `dense` or `sparse` for injector A1 blocks |
| `kluThreshold` | int | no | 500 | Min injector size to switch from LAPACK to KLU |
| `numThreads` | int | no | `OMP_NUM_THREADS` | OpenMP thread count |
| `fnormtolAlg` | double | no | 1e-4 | Convergence tolerance for algebraic restoration WRMS norm |

---

## 9. Implementation Phasing

### Phase 1 — Sequential Schur Baseline

- Implement `SolverDDM` with all Schur machinery; disable `#pragma omp` directives
- Validate numerically against `SolverIDA` on IEEE 39-bus and Nordic test networks
- Confirm BDF-2 LTE step control produces trajectories within tolerance of IDA

### Phase 2 — OpenMP Parallelism

- Enable `#pragma omp parallel for` in `factorizeInjectorBlocks()` and
  `backSubstituteAllInjectors()`
- Measure strong scaling (2 / 4 / 8 / 16 threads) on the Nordic test network
- Each `SubDomainDDM` is owned exclusively by one thread — no synchronisation needed

### Phase 3 — Jacobian Aging and Reuse

- Use `jacAge_` / `jacMaxAge_` to skip Jacobian re-evaluation across Newton iterations
  (modified Newton strategy, typically `jacMaxAge = 3–5`)

### Phase 4 — EP Early-Stopping

- Add per-sub-domain convergence flag: freeze contributions of converged injectors to the
  reduced RHS
- Skip `S_i` recomputation for converged sub-domains (asynchronous Schur update)
- Validate EP variant matches Phase 1 baseline numerically

---

## 10. Key Design Decisions

| Decision | Rationale |
|----------|-----------| 
| Dense Eigen + LAPACK for injectors | Injectors have 10–200 variables; `dgetrf` outperforms KLU below ~500 variables |
| KLU for the network system | Already in the SuiteSparse dependency stack; reuses the `klu_analyze`/`klu_factor` pattern established by `SolverIDA` |
| No KINSOL anywhere | Eliminates KINSOL wrapper overhead; hand-rolled loop gives direct control over convergence criteria and Jacobian reuse |
| BDF-2 only (BDF-1 as one-step startup) | Matches the reference method; avoids the variable-order complexity of full IDA BDF1–5 |
| History flush on `reinit()` | Prevents stale `y[n-1]` from polluting the BDF-2 derivative estimate after a topology change |
| `kReduceStep_` as LTE clamp | Reuses a familiar parameter name; semantics shift from fixed multiplier to factor bound — document clearly in PAR file comments |
| F-scaling in `runAlgebraicNewtonLoop` | Cherry-picked from `SolverKINAlgRestoration::solveStrategy()`: builds a diagonal scale from the initial residual to improve WRMS convergence detection for large-magnitude residuals; ~10 lines, zero SUNDIALS dependency |
