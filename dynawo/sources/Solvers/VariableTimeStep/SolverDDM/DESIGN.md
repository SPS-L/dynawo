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
Jacobian factorisation strategy.  Both share the outer `Solver::Impl`
infrastructure (parameter handling, `evalZMode`, the `reinit()` flow).
**`setupNewAlgRestoration` is *not* shared**: at
`Solvers/Common/DYNSolverImpl.h:340` it is `override = 0` (pure virtual on
the `Solver::Impl` base), and both `SolverIDA::setupNewAlgRestoration` and
`SolverCommonFixedTimeStep::setupNewAlgRestoration` provide ~100 LOC
implementations of their own.  SolverDDM must therefore clone the
fixed-step pattern (instantiate its own `SolverKINAlgRestoration` instance,
plumb the algebraic-equation residual into it, handle the `ALGEBRAIC_MODE`
and `ALGEBRAIC_J_UPDATE_MODE` variants).  See §5.3 for the implementation
sketch and §11 for the LOC budget.

Placing DDM next to IDA makes dependency management, testing, and CMake
integration straightforward.

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

void SolverDDM::init(const boost::shared_ptr<Model>& model, double t0, double tEnd) {
    Solver::Impl::init(t0, model);        // sets up y[], yp[] via getY0
    auto* mm = dynamic_cast<ModelMulti*>(model.get());
    if (!mm) throw DYNError(Error::SOLVER_ALGO, SolverDDMInvalidModel);
    buildDomainDecomposition(mm);
}
```

The **network sub-model** is identified by `subModel->modelType() == "NETWORK"`
(see `Models/CPP/ModelNetwork/DYNModelNetwork.cpp:130` — the C++ network model
is constructed as `ModelCPP("NETWORK")`).  All other sub-models are treated as
injectors.

### 2.2 Required additions to DYNModelMulti.h

`ModelMulti` must expose a read-only view of its sub-model list.  Add:

```cpp
// DYNModelMulti.h — new public method (non-virtual, no ABI impact on Solver)
// NB: the existing private field at DYNModelMulti.h:599 is
//   std::vector<boost::shared_ptr<SubModel> > subModels_;
// so the accessor must return that exact type — the rest of the codebase
// uses boost::shared_ptr<SubModel>, not std::shared_ptr.
const std::vector<boost::shared_ptr<SubModel> >& getSubModels() const;
```

Implemented trivially in `DYNModelMulti.cpp` by returning the existing private
`subModels_` vector.

No changes to `DYNModel.h` (the virtual `Model` interface) are required because
`SolverDDM::init()` explicitly targets `ModelMulti` — exactly as `SolverIDA`
accesses SUNDIALS context without `Model` knowing about it.

### 2.3 Required `SubModel` API additions (prep PR — out of DDM scope)

Several pieces of information needed by the BBD block extractor are not
currently exposed by `SubModel`.  These additions are useful independently of
DDM (e.g. for connector diagnostics, profiling, partial-Jacobian work) and
should land in a separate, focused PR **before** SolverDDM is built on top:

```cpp
// DYNSubModel.h — additions (all default-implemented for backwards compat)

/**
 * @brief Return the global y[] index pairs (Vr, Vi) of every terminal bus
 *        this sub-model is electrically connected to.
 *
 * Required so that DDM can locate the (≤4) non-zero columns of Bᵢ and rows
 * of Cᵢ without parsing the global sparse Jacobian.  Default returns an
 * empty vector (non-injector sub-models such as ModelOmegaRef).
 */
virtual std::vector<std::pair<int,int> > getBusTerminalIndices() const {
  return {};
}

/**
 * @brief Number of interior (state) variables, i.e. those whose residuals
 *        live entirely inside this sub-model and do not couple directly to
 *        bus voltages.  See §2.4 for the precise classifier.
 */
virtual int getInteriorVariableCount() const { return 0; }
```

The first method is the load-bearing one for DDM: without it, recovering the
`busVoltageIdx[k]` mapping requires reverse-engineering the network sparsity
pattern at every step.  The second is convenient but can be derived from
`getYType()` (§2.4) if the prep PR slips.

### 2.3.1 Extracting per-sub-model Jacobian blocks

`SubModel::evalJtSub(t, cj, rowOffset, jt)` fills rows
`[fDeb(), fDeb()+sizeF())` of the global sparse `jt`.  To extract the local
blocks `Aᵢ¹, Aᵢ², Aᵢ³, Aᵢ⁴, Bᵢ, Cᵢ`:

1. **Allocate a local `SparseMatrix` of size `sizeF(i) × sizeY(i)`** (plus the
   small coupling columns).
2. Call `subModel->evalJt(t, cj, 0, localJt)` with `rowOffset = 0`.
3. Extract the sub-blocks by column ranges:
   - Columns `[0, nᵢᵢⁿᵗ)` → `Aᵢ¹` (interior–interior) and `Aᵢ³` (interface–interior).
   - Columns `[nᵢᵢⁿᵗ, nᵢᵢⁿᵗ + nᵢᵉˣᵗ)` → `Aᵢ²` (interior–interface) and `Aᵢ⁴`.
   - Columns corresponding to `busVoltageIdx[k]` (from `getBusTerminalIndices()`)
     in the **global** `y[]` → `Bᵢ` rows.
4. `Cᵢ` is extracted from the **network** sub-model Jacobian: columns
   `[yDeb(i), yDeb(i)+sizeY(i))` within the network block `D`.

The interior/interface partition is determined by scanning
`subModel->getYType()` — see §2.4 for the precise classifier.

```cpp
struct SubDomainDDM {
    boost::shared_ptr<SubModel> subModel;   // matches ModelMulti::subModels_

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

`getYType()` (declared at `Modeler/Common/DYNSubModel.h:571`, enum in
`Common/DYNEnumUtils.h:32-38`) returns **four** values, not two:
`DIFFERENTIAL`, `ALGEBRAIC`, `EXTERNAL`, `OPTIONAL_EXTERNAL`.

A binary "DIFFERENTIAL = interior, anything-else = interface" split is wrong:
ordinary algebraic state variables of an injector (e.g. the algebraic glue
between rotor and stator equations of a synchronous machine) are interior to
the sub-domain and must end up in `Aᵢ¹`, not `Aᵢ⁴`.

The correct classifier is:

- **Interface** variables (live in `Aᵢ⁴`, contribute to `Bᵢ`/`Cᵢ`): those
  whose `propertyContinuousVar_t` is `EXTERNAL`.  These are the variables the
  sub-model imports from the global state (typically bus voltages or
  shared system frequency); their residuals are written by another sub-model
  (the network, or `ModelOmegaRef`).  `OPTIONAL_EXTERNAL` is treated as
  interface **if and only if** the corresponding connector is wired in the
  current scenario — confirm at `init()` time via `SubModel`'s
  connection-info accessor; otherwise treat as interior.
- **Interior** variables (live in `Aᵢ¹`): `DIFFERENTIAL` plus the
  injector-local `ALGEBRAIC` variables (those that are not `EXTERNAL`).

```cpp
void SolverDDM::classifyVariables(SubDomainDDM& sd) {
    const propertyContinuousVar_t* yType = sd.subModel->getYType();
    sd.nInt = 0;
    sd.nExt = 0;
    for (int j = 0; j < sd.nTotal; ++j) {
        switch (yType[j]) {
            case DIFFERENTIAL:
            case ALGEBRAIC:
                ++sd.nInt;       // interior to the sub-domain
                break;
            case EXTERNAL:
                ++sd.nExt;       // genuine interface
                break;
            case OPTIONAL_EXTERNAL:
                // Interface iff the optional connection is actually wired;
                // see SubModel connector-info accessors. Default: interior.
                if (sd.subModel->isOptionalExternalConnected(j)) ++sd.nExt;
                else                                            ++sd.nInt;
                break;
            default:
                throw DYNError(Error::SOLVER_ALGO,
                               SolverDDMUnknownYType, sd.subModel->name(), j);
        }
    }
    // Interior indices come first; reordering handled in extractLocalJacobian()
}
```

(`isOptionalExternalConnected` is part of the prep-PR API surface described
in §2.3.  If that PR has not landed, fall back to "treat
`OPTIONAL_EXTERNAL` always as interface" and document the conservatism in
the parameter file.)

### 2.5 Connector rows outside the BBD structure

The BBD decomposition assumes every off-diagonal coupling flows through
either the network buses (handled by `Bᵢ`/`Cᵢ`) or BDF history (handled by
`cj`).  This is **not** the full picture: `ModelMulti::evalJt` at
`Modeler/Common/DYNModelMulti.cpp:425-444` calls

```cpp
connectorContainer_->evalJtConnector(jt);
connectorContainer_->evalJtPrimConnector(jtPrim);
```

after the per-sub-model `evalJtSub` pass.  These contributions implement
Dynaωo's flow and continuous *yConnectors* — equations whose row spans
columns from **two or more** sub-models simultaneously.  Concrete cases
present in the RTE PFR snapshots include:

- `ModelOmegaRef` aggregating rotor speeds across all on-line synchronous
  machines into a single system-frequency reference.
- `ModelCentralizedShuntsSectionControl` switching shunt sections based on
  a voltage state shared by multiple buses.
- `ModelSecondaryVoltageControlSimplified` (SVR) — pilot-bus voltage
  coupled to multiple participating generators.
- HVDC link controls coupling rectifier and inverter station state.

**Implications for SolverDDM**

These rows are *neither* injector-interior *nor* expressible as
rank-2/rank-4 updates to the bus block — they span arbitrary subsets of
sub-model states.  The prototype's options, in order of increasing
ambition:

1. **Assumption (prototype scope):** assume the test case contains *no*
   cross-injector connectors.  Verify at `init()` by asserting
   `connectorContainer_->nbContinuousConnectors() == 0` for the continuous
   block that DDM operates on.  **The Nordic case satisfies this; PFR and
   the RTE_snapshots corpus do not** — every PFR job loads `ModelOmegaRef`
   and at least one SVR. So this option is for development bring-up only.
2. **Schur-extend the network solve:** treat connector rows as additional
   rows of the reduced system `D̃ ΔV = −g̃`.  The connector Jacobian
   contributions are dense in the columns of the participating sub-models'
   *interface* variables (which we already know), so they fold cleanly into
   the rank-update accumulation `D̃ = D − Σᵢ C̃ᵢ Bᵢ + J_connector`.
   Connector rows that touch *interior* variables (rare, but check
   `ModelOmegaRef`'s formulation) require an additional Schur pass — these
   sub-models must be treated as a single coupled block rather than a list
   of independent injectors.
3. **Full Schur-of-Schurs:** treat the connector block as a third level in
   the hierarchy. Out of scope for the first prototype.

The prototype targets option (1) on Nordic for bring-up, then upgrades to
(2) before any PFR-scale benchmark.  **The realistic LOC estimate in §11
includes (2); (3) is a follow-up.**

**Test cases that violate option (1):**

- `PFR_20240605_N_NB_all_retained` — primary RTE benchmark, ~6000 buses,
  contains `ModelOmegaRef` and SVR.
- `PFR_20240605_events` — extended event variant.
- All 61 `RTE_snapshots/operating_point_*` cases.

Only `Nordic` is connector-free for SolverDDM bring-up purposes.

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
          if (sd.nInt > 0):
            // Eigen::FullPivLU::solve() on a 0×0 matrix is undefined behaviour,
            // so the interior back-substitution must be guarded.  Algebraic-only
            // sub-models (e.g. ModelLoadRestorativeWithLimits with no internal
            // state) hit this path.  luA1 is never .compute()'d in that case.
            Δxᵢᵢⁿᵗ = luA1.solve(−fᵢᵢⁿᵗ − A2ᵢ Δxᵢᵉˣᵗ)
          else:
            Δxᵢᵢⁿᵗ = (no-op)

  (3.4) Update: y[yDeb(i)..yDeb(i)+nTotal) += Δxᵢ
                y[network_range]           += ΔV

  (3.5) Convergence check per injector:
        ‖Δxᵢ‖ < tol_x  AND  ‖fᵢ(y_updated)‖ < tol_f  → mark converged
        (even if converged, recompute ‖fᵢ‖ each subsequent iter using new V)

  (3.6) Global convergence: all injectors converged AND ‖ΔV‖ < tol_V

  (3.7) Step failure: if k == kmax-1 and not converged → halve h, retry
```

**Parallel-region aliasing rules.** Steps 3.1 and 3.3 are declared
`#pragma omp parallel for` across injectors `i`.  Each iteration touches
**only** `injectors_[i]`'s own `SubDomainDDM` — never another's.  In
particular:

- `Eigen::FullPivLU<MatrixXd>` is *not* thread-safe across instances that
  share state, and its `.compute()` and `.solve()` allocate internal
  permutation matrices on the heap.  The `luA1` / `luSi` members of
  `SubDomainDDM` are therefore stored **per injector** — never as a single
  shared instance, never aliased.
- `injectors_` is sized once in `init()` and never resized during a step;
  threads may safely index it.
- The reduction into the network block in step 3.2 is **serial** (see §6.2
  on `klu_common` lifetime) so no cross-injector reduction races occur.
- **NUMA / first-touch**: `SubDomainDDM` objects (including the `Eigen`
  dense blocks) should be constructed in `init()` *from the same OMP thread
  that will own injector `i`'s parallel iteration*, so the dense storage is
  pinned to the right socket.  Practically this means pre-binding the
  parallel-for schedule (e.g. `static` chunk size 1 with a fixed thread
  count) and allocating from the bound thread.

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

**Effective integration order on event-heavy benchmarks.**  The PFR primary
benchmark (`PFR_20240605_N_NB_all_retained`) has scheduled discrete events
roughly every 10 s of simulated time over a 4000 s horizon — order ~400
BDF restarts.  In the worst case (every event forces a restart), the
solver spends only the inter-event interval at BDF-2; with average step
sizes in the 50–200 ms range that leaves on the order of 50–200 BDF-2
steps between restarts, so the **effective time-averaged integration order
is between 1.7 and 1.9** rather than the nominal 2.

For long event-free intervals (e.g. the steady-state preamble before the
first scheduled fault, or the `Nordic` test case after the initial
transient) BDF-2 dominates and the order claim holds. **The BDF-2 benefit
should therefore be evaluated on long event-free segments specifically;
the headline "BDF-2 over 4000 s" claim is misleading and is not used as a
performance target.**  See §10 for the validation cases that exercise each
regime.

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
void    init(const boost::shared_ptr<Model>&, double t0, double tEnd) override;
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
    // NB: model_->rotateBuffers() is NOT called here.  rotateBuffers() is the
    // per-accepted-step finaliser (see DYNSolverImpl.cpp:246 and
    // DYNSolverCommonFixedTimeStep.cpp:263/290/476 — every call site is in an
    // accepted-step path).  Calling it once per solve() invocation would
    // desynchronise history buffers when a step is rejected or when solve()
    // is re-entered after a mode change without an accepted step in between.

    double t = tSolve_;
    while (t < tAim) {
        double hTry = std::min(h_, tAim - t);
        bool stepOk = tryStep(t, hTry);   // see §5.2.1 — owns rotateBuffers
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

### 5.2.1 `tryStep()` — accepted-step finalisation

`rotateBuffers()` advances the model's per-sub-model state history one
step forward (`y_n ← y_{n+1}`, dropping the oldest BDF-2 history slot).
This must happen **exactly once per accepted step** and **never** on a
rejected step, otherwise the BDF history is corrupted.  The pattern used by
the other Dynaωo solvers (`SolverIDA`, `SolverSIM`, `SolverTRAP`) is to
call it at the end of the per-step block, after error-test acceptance:

```cpp
bool SolverDDM::tryStep(double t, double h) {
    // ... BBD-Newton iteration, error estimate ...
    if (err > 1.0) {
        return false;            // step rejected: do NOT rotate, do NOT update history
    }
    updateBdfHistory(/* new y_{n+1} */);
    model_->rotateBuffers();     // commit accepted step (mirrors DYNSolverIDA.cpp:759)
    return true;
}
```

On rejection, the Newton solution is discarded and `vectorY_` is reverted
to its pre-step contents; the BDF history buffer (`ypHistory_` and the
optional BDF-2 second-back slot) is left untouched.

### 5.3 `reinit()` and `setupNewAlgRestoration()` — algebraic restoration

Dynaωo's standard flow after a `ModeChange` is for the `Simulation` layer to
call `solver->reinit()`.  `Solver::Impl::setupNewAlgRestoration` is **pure
virtual** (`Solvers/Common/DYNSolverImpl.h:340` declares
`bool setupNewAlgRestoration(modeChangeType_t) override = 0`) — there is no
shared implementation to delegate to.  `SolverIDA` and
`SolverCommonFixedTimeStep` each provide their own ~100 LOC implementation;
SolverDDM must do the same.

The DDM implementation mirrors `SolverCommonFixedTimeStep::setupNewAlgRestoration`
(see `Solvers/FixedTimeStep/DYNSolverCommonFixedTimeStep.{h,cpp}:171-175` for
the construction pattern, `:436` for the function body): construct a
solver-owned `SolverKINAlgRestoration` lazily on the first
`ALGEBRAIC_MODE` event, configure it from the DDM parameter set
(distinct `fnormtolAlg`/`mxiterAlg` vs. `*AlgJ_` parameters for the two
mode-change variants), and run it on the current `vectorY_` to restore
algebraic consistency.

```cpp
// DYNSolverDDM.h (private members)
boost::shared_ptr<SolverKINAlgRestoration> solverKINAlgRestoration_;

// DYNSolverDDM.cpp
bool SolverDDM::setupNewAlgRestoration(modeChangeType_t modeChangeType) {
    if (!solverKINAlgRestoration_) {
        solverKINAlgRestoration_.reset(
            new SolverKINAlgRestoration(printReinitResiduals_));
        solverKINAlgRestoration_->init(model_, SolverKINAlgRestoration::KIN_ALGEBRAIC);
    }
    const bool isJUpdate = (modeChangeType == ALGEBRAIC_J_UPDATE_MODE);
    solverKINAlgRestoration_->setupNewAlgRestoration(
        isJUpdate ? fnormtolAlgJ_ : fnormtolAlg_,
        isJUpdate ? initialaddtolAlgJ_ : initialaddtolAlg_,
        isJUpdate ? scsteptolAlgJ_ : scsteptolAlg_,
        isJUpdate ? mxnewtstepAlgJ_ : mxnewtstepAlg_,
        isJUpdate ? msbsetAlgJ_ : msbsetAlg_,
        isJUpdate ? mxiterAlgJ_ : mxiterAlg_,
        isJUpdate ? printflAlgJ_ : printflAlg_);
    return true;
}

void SolverDDM::reinit() {
    setupNewAlgRestoration(model_->getModeChangeType());
    solverKINAlgRestoration_->solve();   // KINSOL Newton on algebraic block
    // After convergence, mark all DDM caches dirty:
    markAllJacobiansDirty();
    bdfOrder_ = 1;        // reset integration order (§4.1)
    clearHistory();
}
```

The two `KINSOLAlgRestoration` parameter sets (`Alg` vs. `AlgJ`) are the
existing common solver parameters; DDM consumes them via
`defineCommonParameters()` in the same way as `SolverCommonFixedTimeStep`.

**LOC implication**: this clone is ~100–120 LOC of solver-specific code and
is reflected in §11's scope estimate.

### 5.4 `calculateIC()` — initial conditions

```cpp
void SolverDDM::calculateIC(double tEnd) {
    // Reuse the same KINSOL-based IC solve as SolverIDA
    Solver::Impl::calculateIC(tEnd);
    markAllJacobiansDirty();
}
```

### 5.5 `SolverType` enum — no change required

The `SolverType` enum in `Solvers/Common/DYNSolver.h:55-61` is **unused
outside its own definition** (verified by repository-wide grep: no
references to `SolverSundials1`, `SolverSundials2`, or `SolverSimplifie`
anywhere in `dynawo/dynawo/sources/`).  Dynaωo's solver registration is
purely name-based via `dlopen` + `SolverFactory::createSolver(name)`, so
SolverDDM becomes loadable through its CMake target name and the
`desc_solver(...)` macro alone — adding a new enum value would be dead
code.  The enum is left untouched.

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

### 6.2 KLU `klu_common` lifetime

`klu_common` contains mutable state (last error code, workspace pointers,
ordering choices) and is not thread-safe across concurrent users.

DDM operates **two** KLU contexts, mirroring the IDA/KINSOL pattern:

1. **DDM-owned network solve** — a single `klu_common` instance owned by the
   `SolverDDM` object, used for the reduced system `D̃ ΔV = −g̃` in step 3.2.
   The reduction itself is performed serially (per §3 "Parallel-region
   aliasing rules"), so this single context is only ever touched by the
   main thread.
2. **KINSOL-owned algebraic-restoration solve** — the
   `SolverKINAlgRestoration` instance constructed in §5.3 internally
   allocates **its own** KLU context for its KINSOL linear solver
   (`SUNLinSol_KLU`).  This is invisible to DDM and must not be conflated
   with (1).

The earlier draft's "a single `klu_common` per SolverDDM object" claim was
imprecise: it ignored the KINSOL-owned context.  Both contexts coexist;
neither is shared across threads.  If, in a future optimisation, the
network solve itself is parallelised over multiple RHS columns (e.g. for a
sensitivity sweep), each worker thread must allocate its own `klu_common`
clone — but that is out of scope for the initial prototype.

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

Comparison against a `SolverIDA` reference trajectory.  The **primary**
gate is the project-wide numerical-correctness criterion from
`dynawo/performance-analysis/SPRINT_GUIDE.md`:

1. **Relative L2 norm of output curves** (primary): for every recorded
   curve `c`,
   `‖c_ddm − c_ref‖₂ / ‖c_ref‖₂ < 1 × 10⁻⁵`.
   Any curve exceeding this threshold blocks merge.
2. **Final steady-state mismatch**: `‖F(y_final)‖₂ < 1 × 10⁻⁴`.
3. **Algebraic variable consistency** at each saved time point.

Supplementary diagnostics (not gates — used to triage failures):

- **Normalised DTW distance per state** (diagnostic): a useful tool when
  the L2 gate fails due to small time-axis shifts (e.g. event timing
  jitter from BDF order changes).  Threshold `< 0.01` per state was the
  original draft criterion; it is retained as a per-state diagnostic but
  is no longer the merge gate.
- **Per-bus voltage magnitude/angle trajectories** under the L2 metric, to
  localise any failing-curve diagnostic to a region of the network.

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

## 11. Implementation scope

A faithful, code-complete SolverDDM is **not** a 5-file change — it is on
the same order of magnitude as `SolverIDA` (941 + 289 LOC) plus a clone of
the `SolverCommonFixedTimeStep` algebraic-restoration scaffolding (~565
LOC of shared infrastructure that is not directly reusable).

Realistic breakdown, by component:

| Component | Estimated LOC |
|-----------|---------------|
| BDF-1/2 history + step controller (§4) | ~250 |
| BBD block classifier + extractor from per-injector sparse Jacobian (§2.3.1, §2.4) | ~400 |
| Newton loop, Schemes A and B (§3) | ~300 |
| Connector-row Schur extension (§2.5 option 2) | ~300 |
| `setupNewAlgRestoration` clone + DDM-side `reinit()` (§5.3) | ~120 |
| KLU network-solve wrapper + `klu_common` lifetime (§6.2) | ~150 |
| Parameter parsing (`defineSpecificParameters`, defaults — §8) | ~150 |
| Statistics, header/printer methods, end-of-run report (§9) | ~100 |
| Dictionary entries for new error keys / log messages | ~50 (~10–15 strings) |
| `SubModel` API prep PR (§2.3, separately landed) | ~150 |
| CMake target + headers + install rules (§7) | ~80 |
| Unit + integration tests (Google Test, cf. SolverIDA `test/`) | ~400–800 |

**Total: ~2,200–3,200 LOC** for the prototype (lower bound: no connector
Schur, no NUMA pinning, single-threaded; upper bound: PFR-scale with
connector Schur and per-thread NUMA-aware injector blocks).

This is a multi-month effort, not a sprint-sized change.  The prep PR
(§2.3) should land first as it has independent value; the BBD extractor
should be prototyped as a **debug pass** alongside `SolverIDA` (compute
the BBD blocks each step and assert `‖assembled_BBD − global_J‖_F <
1e-10`) before any DDM-specific Newton code is written.

---

## 12. References

1. Aristidou, P. (2015). *Dynamic Simulations of Large-Scale Power Systems Using
   Parallel Processing Techniques*. PhD thesis, Université de Liège. **Chapter 4**.
2. Fabozzi, D. et al. (2012). "Decomposed solution of the ordinary differential
   equations characterizing large-scale power system dynamics." *PSCC 2012*.
3. Davis, T. A. (2010). "Algorithm 907: KLU, a direct sparse solver for
   circuit simulation problems." *ACM TOMS* 37(3).
4. Hindmarsh, A. C. et al. (2005). "SUNDIALS: Suite of Nonlinear and
   Differential/Algebraic Equation Solvers." *ACM TOMS* 31(3).
5. Eigen library: https://eigen.tuxfamily.org
