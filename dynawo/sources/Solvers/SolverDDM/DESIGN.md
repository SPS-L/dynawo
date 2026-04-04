# SolverDDM: Schur-Complement Domain Decomposition Solver for Dynaωo

## Executive Summary

This document provides a comprehensive implementation design for `SolverDDM`, a new Dynaωo solver that applies a **Bordered-Block-Diagonal (BBD) Schur-complement decomposition** to the Newton iterations at each time step. The mathematical foundation is the decomposition method from Fabozzi (2012) and Aristidou & Van Cutsem (2015), specifically the **single-level Schur-complement formulation (Chapter 4 of Fabozzi's thesis)**. The Chapter 5 localisation technique from the same source is intentionally excluded from this baseline implementation because it relaxes spatial accuracy and is incompatible with the solver's correctness guarantee against the reference `SolverIDA`.

The design follows Dynaωo's strict modeller/solver separation: all model code (residual F, Jacobian J, zero-crossings g, and event callbacks) is unchanged; `SolverDDM` only replaces the inner linear algebra step within the Newton loop. The solver maps naturally onto Dynaωo's existing `SubModel` hierarchy — each `SubModel` is a BBD injector, and the shared network (`ModelNetwork`/`ModelBus`) provides the network matrix D. Parallelism across injectors is achieved via OpenMP over the sub-domain solves.

---

## 1. Mathematical Background

### 1.1 BBD Structure of the Jacobian

At each Newton iteration k and time step, Dynaωo must solve the linear system

```
J [Δx₁; ...; Δxₙ; ΔV] = -[f₁; ...; fₙ; g]
```

where the Jacobian has the BBD structure

```
J = | A₁          B₁ |
    |    A₂        B₂ |
    |       ...    .. |
    |          Aₙ  Bₙ |
    | C₁ ... Cₙ  D   |
```

Here `Aᵢ ∈ ℝⁿⁱ×ⁿⁱ` is the Jacobian of injector i's equations with respect to its own state variables `xᵢ`; `Bᵢ ∈ ℝⁿⁱ×²ᴺ` is the sensitivity of injector i's equations to bus voltages; `Cᵢ ∈ ℝ²ᴺ×ⁿⁱ` maps injector currents into the network equations (2-column extraction matrix for the connected bus); and `D ∈ ℝ²ᴺ×²ᴺ` is the network matrix. N is the number of AC buses.

### 1.2 Schur Complement Factorisation

Block Gaussian elimination on the injector rows yields:

```
Δxᵢ = Aᵢ⁻¹(-fᵢ - Bᵢ ΔV)     i=1,...,n     (1)
D̃ ΔV = -g + Σᵢ C̃ᵢ fᵢ                        (2)
```

where the **Schur-corrected network matrix** is:

```
D̃ = D - Σᵢ C̃ᵢ Bᵢ,    C̃ᵢ = Cᵢ Aᵢ⁻¹          (3)
```

The term `C̃ᵢBᵢ` is a sparse **rank-2** (single-port) or **rank-4** (two-port) correction to `D̃`, affecting only the 2×2 (or 4×4) diagonal block of the bus connected to injector i. This means `D̃` inherits the structural sparsity of D and can be re-ordered and factorised with KLU.

### 1.3 Variant: Updated Right-Hand Side

A variant replaces the right-hand side of (1) with a fresh `fᵢ` evaluation at the updated voltage `Vᵏ`:

```
Aᵢ Δxᵢ = -fᵢ(xᵢᵏ⁻¹, zᵢ, Vᵏ)               (1')
```

This approximation requires one additional `fᵢ` evaluation per iteration but significantly accelerates Newton convergence after large discontinuities (e.g., short-circuit inception) because `Vᵏ` is far from `Vᵏ⁻¹`. The accuracy of the final solution is identical to the standard Newton scheme.

### 1.4 Acceleration: Skip Converged Injectors

At each Newton iteration, injectors whose correction vector `‖Δxᵢ‖` has already fallen below tolerance are not re-solved. Once the correction test is satisfied for injector i, it switches to checking only the mismatch norm `‖fᵢ‖`, avoiding repeated factorisations of Aᵢ. This is the main source of per-iteration speedup in large systems where most injectors are in quasi-steady-state.

### 1.5 Acceleration: Lazy Jacobian Update

The Aᵢ and D̃ matrices are **not** updated simultaneously:

- When injector i changes a discrete state `zᵢ`, only `Aᵢ`, `Bᵢ`, and `C̃ᵢ` are recomputed; `D̃` is rebuilt from (3) using cached `C̃ⱼ` for j ≠ i.
- When the network calls for a `D̃` update (e.g., topology change), only `Bᵢ` matrices are refreshed; `Aᵢ` and `C̃ᵢ` are reused.

This asymmetric update strategy avoids the O(n) cost of refactorising all Aᵢ on every network event.

---

## 2. Dynaωo Solver Architecture Fit

### 2.1 Solver Directory Layout

`SolverDDM` is placed as a peer of `SolverIDA` and `SolverSIM` in `dynawo/sources/Solvers/`:

```
dynawo/sources/Solvers/
├── FixedTimeStep/      # SolverSIM: fixed-step order-1 Backward Euler
├── VariableTimeStep/   # SolverIDA: SUNDIALS IDA variable-step BDF
├── SolverDDM/          # ← NEW: Schur-complement BBD
│   ├── CMakeLists.txt
│   ├── DYNSolverDDM.h
│   ├── DYNSolverDDM.cpp
│   ├── DYNSubDomainDDM.h
│   ├── DYNSubDomainDDM.cpp
│   ├── DYNSchurNetworkSolver.h
│   ├── DYNSchurNetworkSolver.cpp
│   └── DESIGN.md       ← this file
└── Common/
    └── DYNSolverImpl.h  # Base class: step(), reinit(), printHeader()
```

The solver CMake target links against `Eigen3`, `SuiteSparse::KLU`, and optionally `OpenMP`.

### 2.2 Interface to ModelMulti

`SolverDDM` interacts with `ModelMulti` through the four methods already mandated by `DYNSolverImpl`:

| Method | Use in SolverDDM |
|---|---|
| `evalF(t, y, yp, F)` | Full residual evaluation for network and injectors |
| `evalJt(t, y, yp, cj, J)` | Full Jacobian fill for Aᵢ, Bᵢ, Cᵢ, D assembly |
| `evalG(t, y, yp, g)` | Zero-crossing detection (unchanged from standard solvers) |
| `callJModelicaSubModel(i)` | Used during sub-domain factorisation and event callbacks |

The decomposition into sub-domains is achieved by calling `ModelMulti::getSubModels()`, which returns the vector of `SubModel*` objects. Each `SubModel` maps to one `SubDomainDDM` instance.

### 2.3 Variable Mapping

At `init()` time, `SolverDDM` inspects the connector graph (`DYNConnector.cpp`) to determine, for each `SubModel`, which continuous variables `xᵢ` are internal and which correspond to injected currents at the connected buses. This populates the Cᵢ structure matrices, which are constructed once and cached.

---

## 3. Class Design

### 3.1 `SubDomainDDM`

```cpp
class SubDomainDDM {
public:
    int         subModelIndex;   // index into ModelMulti::getSubModels()
    int         ni;              // number of state variables
    int         busIdx;          // connected bus (or two buses for 2-port)
    bool        isConverged;     // skip flag for inner Newton
    bool        jacNeedsUpdate;  // lazy update flag

    Eigen::MatrixXd  Ai;         // ni × ni injector Jacobian (dense)
    Eigen::MatrixXd  Bi;         // ni × 2N (sparse: only 2 nonzero cols)
    Eigen::MatrixXd  Ci_tilde;   // 2N × ni = Ci * Ai^{-1} (dense; 2 rows live)
    Eigen::VectorXd  fi;         // ni residual
    Eigen::VectorXd  dxi;        // Newton correction

    Eigen::PartialPivLU<Eigen::MatrixXd> luAi;  // factorisation of Ai

    void factoriseAi();
    void computeCiTilde();       // Ci_tilde = Ci * Ai^{-1}
    void solveForDxi(const Eigen::VectorXd& dV);
    void checkConvergence(double tol_f, double tol_x);
};
```

Because individual injectors in a typical Dynaωo model have 10–200 state variables, dense DGETRF via Eigen is faster than sparse KLU for Aᵢ.

### 3.2 `SchurNetworkSolver`

```cpp
class SchurNetworkSolver {
public:
    SunMatrix*       D_sparse;    // 2N × 2N network Jacobian (CSC, owned by KLU)
    klu_symbolic*    klu_sym;     // KLU symbolic analysis (done once at init)
    klu_numeric*     klu_num;     // KLU numerical factorisation
    klu_common       klu_common;

    void buildDtilde(
        const std::vector<SubDomainDDM>& subdomains);  // D_tilde = D - sum(Ci_tilde * Bi)
    void factoriseKLU();
    void solveForDV(Eigen::VectorXd& rhs);             // in-place KLU back-solve
    void refactoriseKLU();                             // klu_refactor (pattern unchanged)
    bool needsSymbolicRefactor;  // set true only on topology change
};
```

KLU symbolic analysis (ordering, fill-reduction via AMD/COLAMD) is performed once at `init()` and reused across all time steps and Newton iterations. Only numerical refactorisation (`klu_refactor`) is called when `D̃` changes.

### 3.3 `SolverDDM` (top-level)

```cpp
class SolverDDM : public SolverImpl {
public:
    void init(shared_ptr<Model> model, const Timeline& timeline) override;
    void step(double tStart, double tStop, double& tEnd) override;
    void reinit() override;

private:
    std::vector<SubDomainDDM>   subdomains_;
    SchurNetworkSolver          networkSolver_;
    double                      hCurrent_;   // current step size (fixed-step mode)
    int                         order_;      // BDF order (1 or 2)
    int                         maxNewtonIter_ = 10;
    double                      tolF_, tolX_;

    void assembleJacobian();
    void newtonIteration(double t, double h);
    void applyBDFDiscretisation(double h, int order);
    void buildSchurRHS(Eigen::VectorXd& rhs);  // g - sum(Ci_tilde * fi)
};
```

---

## 4. Newton Iteration Algorithm

Each call to `newtonIteration()` implements the following loop, corresponding to the "Accelerated Decomposed Dishonest Newton" (Scheme A) of Fabozzi (2012):

```
Initialise: evaluate fi for all i, evaluate g
For k = 1, 2, ..., maxNewtonIter:
    1. [PARALLEL, OpenMP] For each non-converged subdomain i:
           if jacNeedsUpdate[i]: factorise Ai, compute Ci_tilde
           compute fi(xi^{k-1}, zi, V^{k-1})   [or V^k if using variant 1']
    2. buildSchurRHS: rhs = -g + sum_i(Ci_tilde * fi)
    3. buildDtilde: D_tilde = D - sum_i(Ci_tilde * Bi)
    4. SchurNetworkSolver::factoriseKLU / refactoriseKLU
    5. Solve D_tilde * dV = rhs  →  obtain Delta_V
    6. [PARALLEL, OpenMP] For each non-converged subdomain i:
           solve Ai * dxi = -(fi + Bi * Delta_V)  →  obtain Delta_xi
           update xi^k = xi^{k-1} + dxi
    7. Update V^k = V^{k-1} + Delta_V
    8. Check convergence for each subdomain i:
           if ||dxi|| < tol_x AND ||fi|| < tol_f: mark i as converged
    9. Check global network convergence: ||Delta_V|| and ||g||
    10. If all converged: break
```

The injector factorisations in step 1 and back-solves in step 6 are independent across all i and are parallelised with `#pragma omp parallel for` over the subdomain vector. No shared mutable state exists between subdomain objects.

---

## 5. Event Handling in the DDM Solver

Event handling is the single most important source of solver stalls in large-scale power system DAE simulation: discrete events force the step size to decrease, trigger Jacobian refactorisations, and require algebraic re-initialisation.

### 5.1 The Problem in Context

When a zero-crossing function `gᵢ(x, t)` crosses zero, the solver must:
(a) locate the event time `tₑ` with sufficient precision,
(b) update the discrete state `zᵢ`,
(c) resolve the algebraic jump `V(tₑ⁺)`, and
(d) reinitialise the integration formula.

In large systems with hundreds of controllers, limiters, and protection relays, events fire with high frequency — Fabozzi's PEGASE case P3 records events at nearly every time step over a 200 s simulation window — and this is identified as "the main factor preventing the step size to increase."

The BBD decomposition makes event handling **cheaper** but introduces additional structure: a discrete event in injector i changes only Aᵢ, Bᵢ, and C̃ᵢ; it does not require a global `D̃` refactorisation, provided the lazy update strategy of Section 1.5 is respected.

### 5.2 Zero-Crossing Detection

Zero-crossing detection proceeds identically to `SolverIDA`: the solver evaluates all `gᵢ(x, t)` at each candidate step, and a sign change triggers bisection root-finding. Because all sub-domain `gᵢ` functions are independent, this step can be parallelised over injectors with OpenMP:

```cpp
// In SolverDDM::detectEvents():
#pragma omp parallel for schedule(dynamic)
for (int i = 0; i < subdomains_.size(); ++i) {
    subdomains_[i].evalZeroCrossings(t, gBuffer_.segment(gOffset_[i], gSize_[i]));
}
```

The network zero-crossing functions (e.g., bus voltage magnitude thresholds) are evaluated sequentially since they require the global voltage vector V.

### 5.3 Event-Time Location and Step Adjustment

When a sign change is detected in interval `[tₙ, tₙ₊₁]`, `SolverDDM` applies the **Illinois method** (a variant of regula falsi with guaranteed convergence) to locate `tₑ` to within a configurable tolerance (`eventLocTol`, default 1e-5 s). The step is then shortened to `h₁ = tₑ - tₙ`, the event is processed, and the integration continues from `tₑ` with step `h₂ = tₙ₊₁ - tₑ`.

In the fixed-step BDF-1 mode, a step-size change invalidates the BDF coefficients. `SolverDDM` handles this by resetting the BDF order to 1 (BEM) at the time step immediately following any event:

> *BEM allows integrating over a discontinuity because in the corresponding equation, if the discontinuity takes place at time tⱼ₊₁, the derivative w(tⱼ₊₁) is not used to compute w(tⱼ).*

After one BEM step post-event, order can be raised back to 2 if the step size is stable.

### 5.4 Algebraic Re-Initialisation After a Jump

When a discrete variable `zᵢ` changes (state event), the algebraic variables may be discontinuous: `V(tₑ⁺) ≠ V(tₑ⁻)`. The algebraic solve at the event instant is handled by running a truncated Newton loop (1–3 iterations) using only the network equation and the event-affected injectors:

1. Mark injector `iₑ` (the one that fired) as `jacNeedsUpdate = true`.
2. Run one BBD Newton pass with `maxNewtonIter = 3` and tightened convergence.
3. All non-event injectors reuse their previous Aᵢ, C̃ᵢ from the pre-event step (lazy update).

This is justified by the observation that solving the full algebraic system after a discontinuity provides "illusory accuracy" — the cost is dominated by the KLU back-solve on `D̃`, not by all injector factorisations.

### 5.5 Injector-Local vs. Global Jacobian Updates

The lazy Jacobian update strategy from Section 1.5 is critical around events. The update policy is:

| Event type | Aᵢ update | Bᵢ update | D̃ update |
|---|---|---|---|
| Discrete state change in injector i | Only injector i | Only injector i | Yes (single rank-2 correction to cached D̃) |
| Network topology change (line trip) | No | All i (refresh Bᵢ) | Yes (full rebuild from cached C̃ᵢ) |
| Step-size change (no event) | No | No | No (reuse D̃) |
| BDF order change | No | No | Rebuild (BDF α coefficient changes) |

When injector i calls for an update of Aᵢ, this matrix is recomputed and factorised, and C̃ᵢ is recomputed since it depends on Aᵢ. However, neither Aⱼ for j ≠ i nor the other C̃ⱼ are updated.

### 5.6 Post-Event Convergence Acceleration

After an event, the Newton iteration often requires more iterations to converge because the system state is far from the linearisation point. The **updated RHS variant** (Section 1.3, equation 1') is therefore **always activated in the first Newton pass after any event**, regardless of the global setting. This re-evaluates `fᵢ(xᵢᵏ⁻¹, zᵢ, Vᵏ)` with the freshly solved Vᵏ rather than the stale Vᵏ⁻¹, providing a much better correction for injectors that have a nonlinear response to the voltage jump.

### 5.7 Time Event Scheduling

Time events (externally scheduled disturbances, e.g., fault clearing at t = 1.0 s) are handled by `SolverDDM` through a priority queue of scheduled event times, pre-populated at `init()` from the timeline. At each `step()` call, if a time event lies within `[tₙ, tₙ + h]`, the step is first shortened to `tₑ`, the event is imposed, and the solve proceeds from `tₑ`. This is identical to the mechanism in `SolverSIM` and requires no additional BBD-specific logic beyond the step-length adjustment.

### 5.8 Chattering Prevention

In systems with many limiters (e.g., OELs in long-term simulations), rapid successive events can cause chattering: the step size is reduced to ~1e-4 s and cannot grow. `SolverDDM` implements a **hysteresis counter**: if more than `maxChatterEvents` (default: 5) sign-changes occur in the same `gᵢ` function within a sliding window of `chatterWindow` (default: 0.02 s) of simulation time, the corresponding zero-crossing is temporarily disabled and an `eventSuppressed` flag is logged to the timeline. This mirrors the approach used in `SolverIDA` via SUNDIALS' event suppression API, adapted here for the BBD context.

### 5.9 Re-initialisation After Large Disturbances

After a fault inception (large voltage jump), the first post-event step is taken with `h = h_min` (default: 1e-3 s) and BDF order 1. The `SolverDDM::reinit()` method performs:

1. Reset all sub-domain `isConverged = false`.
2. Mark all Aᵢ and `D̃` as `needsUpdate = true`.
3. Set BDF order to 1, set `h = h_min`.
4. Run one full BBD Newton solve to compute the post-event consistent initial conditions.
5. Resume step-size growth following the controller in Section 6.

---

## 6. Step-Size and BDF Order Control

`SolverDDM` supports two integration modes, switchable via solver parameters.

### 6.1 Fixed-Step BEM (DynaWaltz compatible)

When `fixedStep = true`, the solver uses a constant step size h with BDF order 1 (Backward Euler Method). This is the simplest mode and matches `SolverSIM`. Events trigger step truncation as in Section 5.3, but the step is immediately restored to h after the post-event BEM step.

### 6.2 Variable-Step BDF-1/BDF-2 (DynaSwing compatible)

When `fixedStep = false`, the solver uses a **local error estimator** to control step size:

```
err = ‖h⁻¹(yⁿ⁺¹_BDF2 - yⁿ⁺¹_BEM)‖₂ / tol
```

- If `err > 1`: reject step, retry with `h ← 0.7h`
- If `err < 0.5`: accept step, grow with `h ← min(1.3h, h_max)`

The Jacobian is not refactorised on a step rejection unless Newton itself has failed to converge. In long-term simulations, the maximum step size varies by orders of magnitude — from 1e-3 s near events to 0.5 s in quiescent periods — and the variable-step mode reduces total Newton solves by 3–10× relative to fixed-step.

---

## 7. Parallelism Design

### 7.1 OpenMP Over Sub-Domains

The dominant parallel opportunity is the independence of all injector sub-solves in steps 1 and 6 of the Newton loop. With n injectors and p threads:

```cpp
#pragma omp parallel for schedule(dynamic, 4) num_threads(p)
for (int i = 0; i < n; ++i) {
    if (!subdomains_[i].isConverged) {
        if (subdomains_[i].jacNeedsUpdate) subdomains_[i].factoriseAi();
        subdomains_[i].solveForDxi(dV_current_);
    }
}
```

Dynamic scheduling (`schedule(dynamic,4)`) is preferred over static because injector sizes `nᵢ` vary. A chunk size of 4 balances scheduling overhead with load imbalance.

### 7.2 Thread Safety

All per-subdomain data (Aᵢ, Bᵢ, C̃ᵢ, fᵢ, dxᵢ, LU factorisation) is owned by the `SubDomainDDM` object and accessed only by its assigned thread. The accumulation of Schur corrections into `D̃` (step 3 of the Newton loop) is performed **sequentially after the parallel factorisations**.

For n > 500 injectors, per-thread partial `D̃` contributions can be accumulated in thread-local dense buffers and summed at a reduction step to avoid false sharing on the sparse `D̃` column entries.

### 7.3 Network Solve (Sequential)

The KLU solve on `D̃ ΔV = rhs` is inherently sequential (a single sparse LU back-solve). For N ≤ 5000 buses, this is not a bottleneck. For very large networks (N > 50,000), an iterative solver (GMRES with incomplete LU preconditioner) could be substituted, but this is outside the Phase 1 scope.

---

## 8. Implementation Phases

### Phase 1: Sequential Schur Baseline (Validation)

**Goal**: Numerically identical results to `SolverIDA` on all existing test cases (Nordic32, IEEE-39). No OpenMP yet.

- Implement `SubDomainDDM`, `SchurNetworkSolver`, and `SolverDDM::newtonIteration()` without skip-converged logic.
- Validate `D̃` assembly against a reference full-Jacobian factorisation (`Eigen::SparseLU`).
- Unit-test individual methods: `factoriseAi()`, `computeCiTilde()`, `buildDtilde()`.
- Compare transient trajectories against `SolverIDA` using Dynamic Time Warping distance as accuracy metric.

### Phase 2: OpenMP Parallelisation

**Goal**: Linear speedup with core count on injector factorisations and back-solves.

- Add `#pragma omp parallel for` to the Newton iteration.
- Validate thread safety of the Schur correction accumulation.
- Strong-scaling study on the Nordic test network: 1, 2, 4, 8, 16 threads.

### Phase 3: Acceleration Strategies (Skip + Lazy Jacobian)

**Goal**: Match or exceed the Scheme A speedup reported in Fabozzi (2–7× on PEGASE).

- Implement `isConverged` skip logic (Section 1.4).
- Implement asymmetric lazy Jacobian update (Section 1.5).
- Profile with `perf` / `callgrind` to confirm hotspot shift from Aᵢ factorisations to network solve.

### Phase 4: Variable-Step BDF Integration

**Goal**: Enable `SolverDDM` as an alternative to `SolverIDA` for DynaSwing simulations.

- Implement the BDF-1/BDF-2 error estimator and step controller of Section 6.2.
- Implement `reinit()` for post-event consistent initial conditions.
- Compare against `SolverIDA` on long-duration voltage stability cases (DynaWaltz scenarios).

---

## 9. Key Design Choices and Trade-offs

| Design Choice | Rationale | Risk / Caveat |
|---|---|---|
| Dense Eigen for Aᵢ | Injectors are small (10–200 vars); DGETRF is faster than sparse KLU for dense blocks | For very large injector models (nᵢ > 300), switch to sparse |
| KLU for D̃ | Already in Dynaωo's dependency stack; reuses existing `klu_analyze` from `SolverIDA` | KLU is sequential; for N > 50,000 may need iterative fallback |
| OpenMP over injectors | Data-independent star topology → zero data races; lightweight vs. MPI | Load imbalance for heterogeneous injector sizes; mitigated by dynamic scheduling |
| Fixed-step mode first | Simplifies validation; identical to `SolverSIM` semantics | No adaptive accuracy control; variable-step added in Phase 4 |
| No spatial localisation (Chapter 5) | Preserves full accuracy; Chapter 5 explicitly excluded | Forfeits 2–7× additional speedup available from localisation |
| Updated RHS variant post-event | Faster Newton convergence after discontinuities at cost of one extra fᵢ eval | Slight increase in residual evaluation count; negligible in practice |

---

## 10. Testing and Validation Strategy

All phases should be validated against `SolverIDA` on the standard Dynaωo test cases:

- **Nordic32**: Short-term voltage stability, long-term voltage collapse. Compare DTW distance of rotor angle and bus voltage trajectories.
- **IEEE-39**: Rotor angle stability with fault and line-trip events.
- **ScalableTestGrids**: Scaling study using the open-source Modelica benchmark library, parameterised to generate 2N×2N EHV/HV networks from N=10 to N=200.

For event-handling correctness, specific test cases should include:

- A **fault with clearing** (voltage jump + topology change): validates Sections 5.4 and 5.9.
- An **OEL activation sequence** (rapid discrete events): validates Section 5.8 chattering prevention.
- An **OLTC tap-changer sequence** (time-event driven): validates Section 5.7.

Accuracy acceptance criterion: normalised DTW distance < 0.01 for all continuous state trajectories, consistent with the thresholds used by Fabozzi for Scheme A validation.

---

## References

- P. Fabozzi, *Contributions to time-domain simulation of large power systems*, PhD thesis, University of Liège, 2012.
- P. Aristidou and T. Van Cutsem, "A parallel processing approach to dynamic simulations of combined transmission and distribution systems," *Int. J. Elect. Power Energy Syst.*, vol. 72, pp. 58–65, 2015.
- P. Aristidou and T. Van Cutsem, "Dynamic simulations of combined transmission and distribution systems using parallel processing techniques," *IEEE Trans. Power Syst.*, vol. 31, no. 6, pp. 5014–5025, 2016.
- A. Hindmarsh et al., "SUNDIALS: Suite of nonlinear and differential-algebraic equation solvers," *ACM Trans. Math. Softw.*, vol. 31, no. 3, pp. 363–396, 2005.
- T. A. Davis and E. Palamadai Natarajan, "Algorithm 907: KLU, a direct sparse solver for circuit simulation problems," *ACM Trans. Math. Softw.*, vol. 37, no. 3, 2010.
