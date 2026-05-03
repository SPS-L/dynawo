<!--
SPDX-License-Identifier: MPL-2.0
SolverDDM — Domain Decomposition Method Solver for Dynaωo
Design Document  (rev 2 — post-review corrections)

Branch: 3_performance-analysis-framework
Repository: https://github.com/SPS-L/dynawo

Algorithm reference:
  Aristidou, P. (2015). "Dynamic Simulations of Large-Scale Power Systems
  Using Parallel Processing Techniques." PhD thesis, University of Liège.
  Chapter 4: Single-level BBD-Newton solver (RAMSES).

Dynaωo integration references:
  DYNSolver.h / DYNSolverImpl.{h,cpp}  — common solver base
  DYNSubModel.h                          — per-sub-model API (evalF, evalJt, yDeb, sizeY, ...)
  DYNModelMulti.{h,cpp}                  — aggregated model (getSubModels, evalJt, ...)
  DYNSolverFactory.h                     — dlopen-based plugin mechanism
  Solvers/VariableTimeStep/CMakeLists.txt — home directory for SolverDDM
-->

# SolverDDM — Domain Decomposition Method Solver

## 0. Location in the source tree

`SolverDDM` is a **variable-step** solver and therefore lives alongside
`SolverIDA` inside `VariableTimeStep/`:

```
dynawo/sources/Solvers/
├── Common/                  ← DYNSolver.h, DYNSolverImpl, DYNSolverFactory
├── AlgebraicSolvers/        ← KINSOL wrapper (algebraic restoration, shared)
├── FixedTimeStep/
│   └── SolverSIM/
├── VariableTimeStep/
│   ├── CMakeLists.txt       ← add_subdirectory(SolverDDM) added here
│   ├── SolverIDA/
│   └── SolverDDM/           ← THIS SOLVER
│       ├── CMakeLists.txt
│       ├── DESIGN.md
│       ├── DYNSolverDDM.h
│       ├── DYNSolverDDM.cpp
│       └── test/
└── util/
```

**Rationale**: `SolverIDA` owns its own variable-step BDF integrator (SUNDIALS
IDA); `SolverDDM` implements a self-contained BDF-1/2 integrator with a BBD
Jacobian factorisation strategy.  Both share the same outer `Solver::Impl`
infrastructure (parameter handling, `evalZMode`, `reinit`,
`setupNewAlgRestoration`).  Placing DDM next to IDA makes dependency
management, testing, and CMake integration straightforward.

`SolverDDM` is loaded by the same `SolverFactory`/dlopen mechanism as `SolverIDA`
— no changes to `DYNSolverFactory.h` or `DYNSolver.h` are needed.

---

## 1. Mathematical Background

### 1.1 Dynaωo DAE system

After BDF-k discretisation, the global Newton residual at step n+1 is

    F(y) = f(t_{n+1}, y, (α y − history) / h) = 0        (1)

where `y ∈ ℝᴺᵗᵒᵗ` collects all continuous variables (differential + algebraic)
of all sub-models.  With BDF-1 (k=1): `yp ≈ (y − y_n) / h`, giving the scalar
coefficient `cj = α/h = 1/h`.

`ModelMulti::evalJt(t, cj, jt)` assembles the full global sparse Jacobian

    J = ∂f/∂y + cj · ∂f/∂yp

in Dynaωo's `SparseMatrix` (CSC format) by calling `evalJtSub` on each
`SubModel` in turn.

### 1.2 BBD structure and Schur reduction (Aristidou Ch. 4)

Split sub-models into:
- **Network model** (index 0 in `ModelMulti`): `g(V, ...) = 0`, Jacobian block `D`.
- **Injector sub-models** i = 1…M (generators, loads, etc.): each has its own
  state vector `xᵢ` and is connected to one or two network buses.

After `initSize()` is called by `ModelMulti`, each `SubModel` exposes:
- `yDeb()` — global offset of its continuous variables in the shared `y[]` buffer.
- `sizeY()` — number of continuous variables (interior + interface combined).
- `fDeb()` — global offset of its residual equations in the shared `f[]` buffer.
- `sizeF()` — number of residuals.

The per-injector local Jacobian has the 4-block structure (Thesis Eq. 4.8):

```
  ┌            ┐
  │ Aᵢ¹  Aᵢ²  │   ← Aᵢ¹: ∂fᵢᵢⁿᵗ/∂xᵢᵢⁿᵗ   Aᵢ²: ∂fᵢᵢⁿᵗ/∂xᵢᵉˣᵗ
  │ Aᵢ³  Aᵢ⁴  │   ← Aᵢ³: ∂fᵢᵉˣᵗ/∂xᵢᵢⁿᵗ   Aᵢ⁴: ∂fᵢᵉˣᵗ/∂xᵢᵉˣᵗ
  └            ┘
```

where `xᵢᵢⁿᵗ` are the interior (rotor flux, exciter, ...) states and `xᵢᵉˣᵗ`
are the interface states (typically terminal voltages re-expressed as local
unknowns).  The Schur complement that eliminates the interior states is:

    Sᵢ = Aᵢ⁴ − Aᵢ³ (Aᵢ¹)⁻¹ Aᵢ²                           (2)

The coupling matrices `Bᵢ` and `Cᵢ` (Thesis Eq. 4.7) each have at most 2 (single-
port) or 4 (two-port) non-zero columns/rows corresponding to the bus voltage
indices `busIdx[]`.

The reduced network system (Thesis Eq. 4.11–4.12) is:

    D̃ ΔV = −g̃                                              (3)

    D̃ = D − Σᵢ C̃ᵢ Bᵢ,    C̃ᵢ = Cᵢ (Aᵢ¹)⁻¹ ← only when nᵢᵢⁿᵗ > 0
                                  else Cᵢ Sᵢ⁻¹

    g̃ = g − Σᵢ C̃ᵢ fᵢ

Back-substitution per injector (Thesis Eq. 4.12):

    Δxᵢ = (Aᵢ¹)⁻¹ (−fᵢᵢⁿᵗ − Aᵢ² Δxᵢᵉˣᵗ)                  (4)
    Δxᵢᵉˣᵗ = Sᵢ⁻¹ (−fᵢᵉˣᵗ − Aᵢ³ Δxᵢᵢⁿᵗ − Bᵢ ΔV)            (5)

**Important**: if a sub-model exposes no interior variables (nᵢᵢⁿᵗ = 0, e.g. a
simple load model), Eq. (2) degenerates to `Sᵢ = Aᵢ⁴` and only one LU solve is
needed (Eq. 5).

### 1.3 Jacobian coefficient dependency on `h`

Under BDF-1 the full local Jacobian contribution of injector `i` to the global
`J` is

    Jᵢ = ∂fᵢ/∂xᵢ + cj · ∂fᵢ/∂ẋᵢ,   cj = 1/h

The blocks `Aᵢ¹, Aᵢ², Aᵢ³, Aᵢ⁴` are therefore all functions of `h`.  A change
in step size **always** invalidates the cached LU factorisations and requires a
full Jacobian re-evaluation for all sub-models.  The same applies to BDF-2 where
`cj = (2τ+1)/((τ+1)h)` with `τ = h_n/h_{n-1}`.

---

## 2. Mapping between Dynaωo Concepts and DDM Variables

### 2.1 Identifying the network model and injector sub-models

`ModelMulti` (in `DYNModelMulti.h`) stores all sub-models internally.
`SolverDDM::init()` casts the `Model*` handle to `ModelMulti*` to obtain the
sub-model list:

```cpp
// DYNSolverDDM.cpp
#include "DYNModelMulti.h"

void SolverDDM::init(const std::shared_ptr<Model>& model, double t0, double tEnd) {
    Solver::Impl::init(t0, model);        // sets up y[], yp[] via getY0
    auto* mm = dynamic_cast<ModelMulti*>(model.get());
    if (!mm) throw DYNError(Error::SOLVER_ALGO, SolverDDMInvalidModel);
    buildDomainDecomposition(mm);
}
```

The **network sub-model** is identified by `subModel->modelType() == "ModelNetwork"`
(or a dedicated API to be added — see Section 2.2).  All other sub-models are
treated as injectors.

### 2.2 Required additions to DYNModelMulti.h

`ModelMulti` must expose a read-only view of its sub-model list.  Add:

```cpp
// DYNModelMulti.h — new public method (non-virtual, no ABI impact on Solver)
const std::vector<std::shared_ptr<SubModel>>& getSubModels() const;
```

Implemented trivially in `DYNModelMulti.cpp` by returning the existing private
`subModels_` vector.

No changes to `DYNModel.h` (the virtual `Model` interface) are required because
`SolverDDM::init()` explicitly targets `ModelMulti` — exactly as `SolverIDA`
accesses SUNDIALS context without `Model` knowing about it.

### 2.3 Extracting per-sub-model Jacobian blocks

`SubModel::evalJtSub(t, cj, rowOffset, jt)` fills rows
`[fDeb(), fDeb()+sizeF())` of the global sparse `jt`.  To extract the local
blocks `Aᵢ¹, Aᵢ², Aᵢ³, Aᵢ⁴, Bᵢ, Cᵢ`:

1. **Allocate a local `SparseMatrix` of size `sizeF(i) × sizeY(i)`** (plus the
   small coupling columns).
2. Call `subModel->evalJt(t, cj, 0, localJt)` with `rowOffset = 0`.
3. Extract the sub-blocks by column ranges:
   - Columns `[0, nᵢᵢⁿᵗ)` → `Aᵢ¹` (interior–interior) and `Aᵢ³` (interface–interior).
   - Columns `[nᵢᵢⁿᵗ, nᵢᵢⁿᵗ + nᵢᵉˣᵗ)` → `Aᵢ²` (interior–interface) and `Aᵢ⁴`.
   - Columns corresponding to `busIdx[k]` in the **global** `y[]` → `Bᵢ` rows.
4. `Cᵢ` is extracted from the **network** sub-model Jacobian: columns
   `[yDeb(i), yDeb(i)+sizeY(i))` within the network block `D`.

The interior/interface partition nᵢᵢⁿᵗ is determined by scanning `subModel->getYType()`
for variables with property `DIFFERENTIAL` — these are the interior variables.
Interface variables are `ALGEBRAIC` variables whose global index falls in the
range `[yDeb(i), yDeb(i)+sizeY(i))` **and** also appear in bus-voltage rows of
the network Jacobian.

```cpp
struct SubDomainDDM {
    std::shared_ptr<SubModel> subModel;

    // Global index ranges within the aggregated y[] buffer
    int yGlobalOffset;       // = subModel->yDeb()
    int nTotal;              // = subModel->sizeY()
    int nInt;                // number of interior (DIFFERENTIAL) variables
    int nExt;                // nTotal - nInt  (interface variables)

    // Bus connectivity (at most 2 bus indices for a two-port device)
    int numPorts;            // 1 or 2
    std::array<int,2> busVoltageIdx;  // global y[] indices of terminal Vr, Vi pairs

    // Local dense blocks (small: nInt x nInt, nInt x nExt, nExt x nInt, nExt x nExt)
    Eigen::MatrixXd A1;  // ∂fint/∂xint   (nInt x nInt)
    Eigen::MatrixXd A2;  // ∂fint/∂xext   (nInt x nExt)
    Eigen::MatrixXd A3;  // ∂fext/∂xint   (nExt x nInt)
    Eigen::MatrixXd A4;  // ∂fext/∂xext   (nExt x nExt)

    // Coupling matrices — stored as compact dense (nTotal x 2*numPorts), NOT nTotal x 2N
    Eigen::MatrixXd Bi;      // rows: sizeF(i),  cols: 2*numPorts
    Eigen::MatrixXd Ci;      // rows: 2*numPorts, cols: sizeF(i)

    // Derived quantities
    Eigen::MatrixXd Si;          // Schur complement (nExt x nExt)
    Eigen::MatrixXd A1_inv_A2;   // (A1)^{-1} A2, pre-computed for back-sub

    // LU factorisations (nInt x nInt and nExt x nExt — both small)
    Eigen::FullPivLU<Eigen::MatrixXd> luA1;
    Eigen::FullPivLU<Eigen::MatrixXd> luSi;

    // Lazy-update flags
    bool jacNeedsUpdate = true;
    bool converged      = false;
};
```

**Critical**: `Bi` and `Ci` are `nTotal × 2*numPorts` and `2*numPorts × nTotal`
respectively — **not** `nTotal × 2N`.  The two (or four) non-zero entries per
column/row are extracted from the global sparse Jacobian at positions
`busVoltageIdx[k]`.

### 2.4 Identifying interface variable partition

```cpp
void SolverDDM::classifyVariables(SubDomainDDM& sd) {
    const propertyContinuousVar_t* yType = sd.subModel->getYType();
    sd.nInt = 0;
    for (int j = 0; j < sd.nTotal; ++j)
        if (yType[j] == DIFFERENTIAL) ++sd.nInt;
    sd.nExt = sd.nTotal - sd.nInt;
    // Interior indices come first; reordering handled in extractLocalJacobian()
}
```

---

## 3. Full Newton Iteration Loop

```
for k = 0, 1, ..., kmax-1:

  (3.1) Parallel per-injector:
        For each non-converged injector i:
          a. subModel[i]->evalF(t, UNDEFINED_EQ)
             → fills fLocal_ in global f[] at offset fDeb(i)
          b. If jacNeedsUpdate[i]:
               extractLocalJacobian(i, t, cj, localJt)
               factoriseBlocks(i)          // luA1, luSi
               precomputeCouplings(i)      // C̃ᵢ = Cᵢ Sᵢ⁻¹ (or Cᵢ (A1)⁻¹ if nInt>0)
               jacNeedsUpdate[i] = false

  (3.2) Serial: build reduced network system
        D̃ = D − Σᵢ C̃ᵢ Bᵢ   (accumulate rank-2/4 updates)
        g̃ = g − Σᵢ C̃ᵢ fᵢ   (sign: g̃ is the RHS of (3), negated for KLU solve)

        // KLU solves D̃ ΔV = −g̃
        klu_refactor(D̃, klu_symbolic_, klu_numeric_, &klu_common_);
        klu_solve(klu_numeric_, ΔV, −g̃, &klu_common_);

  (3.3) Parallel per-injector back-substitution (Eqs. 4–5):
        For each injector i:
          Δxᵢᵉˣᵗ = luSi.solve(−fᵢᵉˣᵗ − A3ᵢ Δxᵢᵢⁿᵗ_prev − Bᵢ ΔV_local)
          if nInt > 0:
            Δxᵢᵢⁿᵗ = luA1.solve(−fᵢᵢⁿᵗ − A2ᵢ Δxᵢᵉˣᵗ)

  (3.4) Update: y[yDeb(i)..yDeb(i)+nTotal) += Δxᵢ
                y[network_range]           += ΔV

  (3.5) Convergence check per injector:
        ‖Δxᵢ‖ < tol_x  AND  ‖fᵢ(y_updated)‖ < tol_f  → mark converged
        (even if converged, recompute ‖fᵢ‖ each subsequent iter using new V)

  (3.6) Global convergence: all injectors converged AND ‖ΔV‖ < tol_V

  (3.7) Step failure: if k == kmax-1 and not converged → halve h, retry
```

### 3.1 Scheme selection (Aristidou Schemes A and B)

Two Newton variants are implemented (selected by PAR file parameter
`scheme`, default `B`):

- **Scheme A** (standard decomposed Newton, Thesis §4.3.1): after step 3.4,
  proceed to check convergence.  `fᵢ` in step 3.1a is evaluated with the
  state from the *previous* outer iteration.

- **Scheme B** (updated-RHS, Thesis §4.3.2): after computing `ΔV` in step 3.2,
  immediately re-evaluate `fᵢ(xᵢ, V_new)` for **all** non-converged injectors
  before back-substitution.  This requires one extra `evalF` pass but improves
  convergence for systems with tight voltage–machine coupling.

### 3.2 Lazy Jacobian update policy

| Trigger | Aᵢ¹/Aᵢ²/Aᵢ³/Aᵢ⁴ | Bᵢ, Cᵢ | D̃ rebuild |
|---|---|---|---|
| First step or restart | ✓ all | ✓ all | ✓ |
| h changed (step-size change) | ✓ all | ✗ | ✓ |
| Newton converged slowly (>N_slow iters) | ✓ all | ✗ | ✓ |
| Injector i discrete event (local) | ✓ i only | ✓ i | ✓ |
| Network topology change (line/transformer trip) | ✓ all | ✓ all | ✓ (D updated externally via reinit) |
| Successful step, fast convergence | ✗ (age) | ✗ | ✗ |

**Rationale for h-change row**: `cj = α/h` enters every Aᵢ block (BDF coefficient
multiplies `∂fᵢ/∂ẋᵢ`).  The coupling matrices `Bᵢ` and `Cᵢ` contain
`∂fᵢ/∂V` terms that are independent of `h`, so they do not need refresh.

**Topology change vs. local event**: a topology change is signalled by
`ModeChange` in the solver state (set by `evalZMode` via `model_->modeChange()`).
A local discrete event is signalled by the specific injector's
`subModel->modeChange()` flag.  The two code paths must be distinguished:

```cpp
if (state_.getFlags(ModeChange)) {
    // Network topology may have changed: full rebuild
    markAllJacobiansDirty();
    rebuildNetworkBlock();
} else {
    // Only affected injectors
    for (auto& sd : injectors_)
        if (sd.subModel->modeChange()) sd.jacNeedsUpdate = true;
}
```

---

## 4. BDF Time Integration

### 4.1 Step-size and order control

BDF-1 is used at the first step and after any discontinuity.  BDF-2 is
activated after two consecutive accepted steps with `err < 0.5 * tol_step`.

The normalised local error estimate (Euclidean norm) is:

    err = ‖ (y_{n+1}^{BDF1} − y_{n+1}^{BDF2} ) / (atol + rtol |y_{n+1}|) ‖₂ / √N

Step accepted if `err ≤ 1`.  New step:

    h_new = h · min(facmax, max(facmin, safety · err^{-1/(k+1)}))

with `safety = 0.9`, `facmax = 5.0`, `facmin = 0.2`.

Minimum acceptable step: `minimalAcceptableStep` from the common PAR parameters
(already in `Solver::Impl::defineCommonParameters()`).

### 4.2 `computeYP` integration

Dynaωo's `Solver` interface requires `computeYP(const double* yy)`.  Under
BDF-1 this is simply:

```cpp
void SolverDDM::computeYP(const double* yy) {
    for (int i = 0; i < model_->sizeY(); ++i)
        vectorYp_[i] = (yy[i] - ypHistory_[i]) * cj_;  // cj_ = 1/h
}
```

where `ypHistory_` stores `y_n` (BDF-1) or the BDF-2 history combination.

---

## 5. Dynaωo Interface Implementation

### 5.1 Class hierarchy

```
DYN::Solver  (pure virtual, DYNSolver.h)
  └── DYN::Solver::Impl  (DYNSolverImpl.{h,cpp})
        └── DYN::SolverDDM  (DYNSolverDDM.{h,cpp})
```

`SolverDDM` inherits `Solver::Impl` (exactly as `SolverIDA` does) and overrides:

```cpp
// Mandatory overrides
void    init(const std::shared_ptr<Model>&, double t0, double tEnd) override;
void    calculateIC(double tEnd) override;          // delegate to SolverKINSOL
void    solve(double tAim, double& tNxt) override;  // outer time-step loop
void    reinit() override;                          // algebraic restoration
void    computeYP(const double* yy) override;

// Informational
const std::string& solverType() const override;
void  defineSpecificParameters() override;
void  setSolverSpecificParameters() override;
void  printHeaderSpecific(std::stringstream&) const override;
void  printSolveSpecific(std::stringstream&) const override;
void  printEnd() const override;
const std::string& getName() override;
```

### 5.2 `solve()` outer loop

```cpp
void SolverDDM::solve(double tAim, double& tNxt) {
    state_.reset();
    model_->reinitMode();
    model_->rotateBuffers();

    double t = tSolve_;
    while (t < tAim) {
        double hTry = std::min(h_, tAim - t);
        bool stepOk = tryStep(t, hTry);
        if (stepOk) {
            t += hTry;
            ++stats_.nst_;
            adaptStepSize(true);
        } else {
            adaptStepSize(false);
            if (h_ < minimalAcceptableStep_)
                throw DYNError(Error::SOLVER_ALGO, SolverDDMStepTooSmall, t);
        }
        // Discrete variable + mode update
        std::vector<state_g> G0(model_->sizeG()), G1(model_->sizeG());
        model_->evalG(t, G0);
        if (evalZMode(G0, G1, t)) {
            state_.setFlags(ModeChange);
            // reinit() will be called by the Simulation layer if ALGEBRAIC_MODE
        }
    }
    tNxt = t;
    tSolve_ = tNxt;
}
```

### 5.3 `reinit()` — algebraic restoration after mode change

Dynaωo's standard flow after a `ModeChange` is for the `Simulation` layer to
call `solver->reinit()`.  `SolverDDM::reinit()` delegates to the shared KINSOL-
based algebraic restoration (already in `Solver::Impl::setupNewAlgRestoration`):

```cpp
void SolverDDM::reinit() {
    // Standard Dynaωo algebraic restoration via KINSOL
    Solver::Impl::setupNewAlgRestoration(vectorY_.data());
    // After convergence, mark all DDM caches dirty
    markAllJacobiansDirty();
    bdfOrder_ = 1;        // reset integration order
    clearHistory();
}
```

### 5.4 `calculateIC()` — initial conditions

```cpp
void SolverDDM::calculateIC(double tEnd) {
    // Reuse the same KINSOL-based IC solve as SolverIDA
    Solver::Impl::calculateIC(tEnd);
    markAllJacobiansDirty();
}
```

### 5.5 `SolverType` enum

Add `SolverDDM = 3` to the `SolverType` enum in `DYNSolver.h`:

```cpp
typedef enum {
    SolverSimplifie = 0,
    SolverSundials1 = 1,
    SolverSundials2 = 2,
    SolverDDM       = 3    // <-- new
} SolverType;
```

---

## 6. Numerical Robustness

### 6.1 Near-singular local Jacobians

`Eigen::FullPivLU` is used for both `luA1` and `luSi` (instead of
`PartialPivLU`) to detect rank deficiency:

```cpp
luA1.compute(A1);
if (luA1.rank() < sd.nInt) {
    Trace::warn() << DYNLog(SolverDDMSingularA1, sd.subModel->name()) << Trace::endline;
    // Force full Jacobian update next iteration; try step-halving
    sd.jacNeedsUpdate = true;
    return STEP_FAILED;
}
```

### 6.2 KLU thread safety

`klu_common` contains mutable state (last error code, workspace pointers).  A
single `klu_common` instance is allocated per `SolverDDM` object — not shared
across threads.  If the network solve is later parallelised over multiple RHS,
each thread must own its own `klu_common`.

### 6.3 Jacobian aging reset on step-size change

```cpp
void SolverDDM::onStepSizeChange(double h_new) {
    cj_ = computeCj(h_new, bdfOrder_);
    markAllJacobiansDirty();   // cj enters all Aᵢ blocks
    // Bᵢ, Cᵢ do NOT need update (no cj dependency)
}
```

---

## 7. CMake Integration

### 7.1 `SolverDDM/CMakeLists.txt`

```cmake
set(SOLVER_DDM_VERSION_STRING ${DYNAWO_VERSION_STRING})
set(SOLVER_DDM_VERSION_MAJOR  ${DYNAWO_VERSION_MAJOR})

set(SOLVER_DDM_SOURCES
    DYNSolverDDM.cpp
)
set(SOLVER_DDM_INCLUDE_HEADERS
    DYNSolverDDM.h
)

add_library(dynawo_SolverDDM SHARED ${SOLVER_DDM_SOURCES})

target_include_directories(dynawo_SolverDDM
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INCLUDEDIR_NAME}>
    $<TARGET_PROPERTY:dynawo_SolverCommon,INTERFACE_INCLUDE_DIRECTORIES>
  PRIVATE
    $<TARGET_PROPERTY:dynawo_API_PAR,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:dynawo_ModelerCommon,INTERFACE_INCLUDE_DIRECTORIES>
)
target_include_directories(dynawo_SolverDDM SYSTEM
  PRIVATE
    $<TARGET_PROPERTY:Boost::boost,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(dynawo_SolverDDM
  PRIVATE
    dynawo_Common
    dynawo_SolverCommon
    dynawo_SolverKINSOL        # algebraic restoration + IC
    SuiteSparse::KLU           # network Schur solve
    SuiteSparse::AMD
    SuiteSparse::COLAMD
    Eigen3::Eigen              # small dense blocks
)

set_target_properties(dynawo_SolverDDM
    PROPERTIES
        VERSION   ${SOLVER_DDM_VERSION_STRING}
        SOVERSION ${SOLVER_DDM_VERSION_MAJOR}
        PREFIX    "")

install(TARGETS dynawo_SolverDDM EXPORT dynawo-targets DESTINATION ${LIBDIR_NAME})
install(FILES ${SOLVER_DDM_INCLUDE_HEADERS}   DESTINATION ${INCLUDEDIR_NAME})

desc_solver(dynawo_SolverDDM)

if(BUILD_TESTS OR BUILD_TESTS_COVERAGE)
    add_subdirectory(test)
endif()
```

### 7.2 `VariableTimeStep/CMakeLists.txt` — addition

```cmake
add_subdirectory(SolverIDA)
add_subdirectory(SolverDDM)   # <-- add this line
```

---

## 8. Solver Parameters (PAR file)

In addition to all **common** parameters inherited from `Solver::Impl`
(`fnormtolAlg`, `minimalAcceptableStep`, etc.), `SolverDDM` adds:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `hMin` | `double` | `1e-6` | Minimum time step (overrides common minimalAcceptableStep if set) |
| `hMax` | `double` | `1.0` | Maximum time step |
| `hStart` | `double` | `1e-3` | Initial time step |
| `tolX` | `double` | `1e-6` | Per-injector Newton state tolerance |
| `tolF` | `double` | `1e-4` | Per-injector residual tolerance |
| `tolV` | `double` | `1e-6` | Network voltage Newton tolerance |
| `maxNewtonIter` | `int` | `50` | Max BBD-Newton iterations per step |
| `scheme` | `string` | `"B"` | Newton scheme: `"A"` or `"B"` (Aristidou §4.3) |
| `jacAgeMax` | `int` | `5` | Max Newton steps before forced Jacobian refresh |
| `bdfMaxOrder` | `int` | `2` | Maximum BDF order (1 or 2) |
| `nSlowStepsBeforeJacUpdate` | `int` | `3` | Slow-convergence threshold |
| `openmpThreads` | `int` | `0` | OMP threads (0 = auto) |

---

## 9. Statistics and Diagnostics

`printEnd()` reports (in addition to the standard common stats):

```
--- SolverDDM Execution Statistics ---
Time steps attempted   : nst_
Time steps accepted    : nst_accepted_
Newton iterations total: nni_
Jacobian evaluations   : nje_
Step rejections        : netf_
Avg Newton iters/step  : nni_ / nst_accepted_
Max injector backlog   : max_non_converged_injectors_
```

---

## 10. Validation Strategy

### 10.1 Accuracy metrics

Comparison against a `SolverIDA` reference trajectory:

1. **State variable trajectories**: normalised DTW distance < 0.01 per state.
2. **Bus voltage trajectories**: magnitude and angle at all network buses, same DTW criterion.
3. **Final steady-state mismatch**: `‖F(y_final)‖₂ < 1e-4`.
4. **Algebraic variable consistency** at each saved time point.

### 10.2 Performance metrics

1. Total CPU time vs. SolverIDA baseline.
2. Newton iteration count per time step (mean ± std).
3. Jacobian evaluation count.
4. Step rejection rate.

### 10.3 Test cases by phase

| Phase | Test case | Purpose |
|---|---|---|
| 1 | IEEE 39-bus, 3-phase fault | Basic BBD Newton, Schemes A & B |
| 1 | IEEE 39-bus, generator trip | Local event update path |
| 2 | Nordic32, voltage collapse | Slow dynamics, step-control |
| 2 | Nordic32, line trip | Topology change → full Jacobian rebuild |
| 3 | IEEE 39-bus, OEL activation | Chattering / rapid discrete events |
| 3 | Nordic32, OEL activation | Multiple simultaneous discrete events |
| 4 | IEEE 39-bus, 4/8/16 OMP threads | Parallel scaling |

---

## 11. References

1. Aristidou, P. (2015). *Dynamic Simulations of Large-Scale Power Systems Using
   Parallel Processing Techniques*. PhD thesis, Université de Liège. **Chapter 4**.
2. Fabozzi, D. et al. (2012). "Decomposed solution of the ordinary differential
   equations characterizing large-scale power system dynamics." *PSCC 2012*.
3. Davis, T. A. (2010). "Algorithm 907: KLU, a direct sparse solver for
   circuit simulation problems." *ACM TOMS* 37(3).
4. Hindmarsh, A. C. et al. (2005). "SUNDIALS: Suite of Nonlinear and
   Differential/Algebraic Equation Solvers." *ACM TOMS* 31(3).
5. Eigen library: https://eigen.tuxfamily.org
