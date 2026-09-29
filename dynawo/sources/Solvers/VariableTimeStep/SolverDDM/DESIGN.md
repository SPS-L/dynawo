<!--
SPDX-License-Identifier: MPL-2.0
SolverDDM, a single-level Schur-complement solver for Dynawo.
Design, third revision, 2026-09-28. Design only: no code exists for it.
Branch: 11_ddm_solver of SPS-L/dynawo, cut from master at ccb4c615d0c.
-->

# SolverDDM: a single-level Schur-complement solver for Dynawo

This document is the third revision of the design and replaces the second revision in full. The second revision was written against the source tree as it was believed to be rather than as it is, and section 0 lists the statements it made that the code contradicts. Every fact about Dynawo below was read from the tree at `ccb4c615d0c`, and every measured figure is quoted from the performance reports kept in the TRAISIM repository with the report named beside it. No simulation was run for this revision, so every projected effect is an estimate and is marked as one.

The proposed solver applies the parallel Schur-complement-based domain decomposition method (DDM) of Aristidou, Fabozzi and Van Cutsem [1, 2] to the differential-algebraic equation (DAE) system that Dynawo assembles from its submodels. The network model forms the central sub-domain, every injector and twoport forms a satellite sub-domain, and the interface unknowns are updated by one sparse solve of the reduced network system per Newton iteration. The satellites are factorised and solved independently and in parallel, their converged members are skipped, and their inactive members may be replaced by a linear sensitivity model. Time integration is a variable-step second-order backward differentiation formula (BDF), and discrete events are handled by repeating the step with the new discrete state rather than by a separate algebraic restoration, which section 7 evaluates against Dynawo's present approach.

The document is organised as follows. Section 1 states the verdict against the real-time goal and the measurements it rests on. Section 2 records how Dynawo presents the system to a solver. Sections 3 to 6 give the decomposition, the formulation, the integration scheme and the Newton scheme. Section 7 compares the two event-handling approaches. Sections 8 to 12 cover the linear algebra, the interface with the rest of Dynawo, the parameters, the statistics and the validation plan. Section 13 sets the scope and the go/no-go experiments, and section 14 lists the decisions left to the author.

## 0. What the second revision got wrong

The earlier text placed the injector-to-network coupling inside the injector's own Jacobian rows, with the bus voltage indices appearing as columns of the injector block and a coupling matrix to be extracted from those columns. In the tree, an injector's residual rows reference only its own continuous variables; the terminal voltage and current are local copies, typed `EXTERNAL` and `ALGEBRAIC` respectively, and the coupling to the bus is carried by connector equations appended after the last submodel's rows (`Modeler/Common/DYNConnector.cpp:488-601`). The two `SubModel` additions it required for locating terminal indices are therefore unnecessary: the connector container already holds every pairing with global indices.

It held that a change of step size invalidates every satellite block and forces a full re-evaluation. The compiled models can return the two parts of the Jacobian separately, since `ModelManager::evalJt` and `evalJtPrim` compute `∂F/∂y + c ∂F/∂ẏ` and `∂F/∂ẏ` from one automatic-differentiation pass (`Modeler/ModelManager/DYNModelManager.cpp:320-383`), so a step-size change costs a re-algebraisation and a refactorisation of stored blocks, not a differentiation. It also held that a network topology change forces every satellite block to be rebuilt; only the central block changes.

It cloned Dynawo's algebraic restoration, an extra nonlinear solve on the algebraic equations at the event instant with its own Jacobian and factorisation. The reference algorithm does not restore; it repeats the step, and section 7 argues that the DDM should do the same. It estimated the step from a local error test comparing the first- and second-order solutions, while the reference solver on the benchmark takes fixed 1 s implicit Euler steps with no error test; an error-controlled step would take many more steps than the benchmark and the real-time comparison would be lost. It depended on Eigen, which is absent from the third-party tree; so is LAPACK. It set `vectorYp_` for every variable in `computeYP`, whereas only differential variables carry a derivative. And it named a validation plan on IEEE 39-bus and Nordic32 cases that the project does not run, in place of the regression harness and the RTE cases that it does.

## 1. Verdict against the real-time goal

The DDM does not bring the RTE cases into real time by parallelism, because the work it parallelises is a small fraction of what breaches the budget. On the 400 s benchmark the `NETWORK` submodel holds 294,902 of 321,787 continuous variables, 293,654 of 316,364 submodel equations and 715,107 of 792,639 Jacobian nonzeros, 90.2 %, in one atomic evaluation (`reports/parallel_evaluation_2026-09-20.md`). The 1,355 other submodels hold 26,885 variables between them. A Jacobian evaluation on an RTE operating point costs about 624 ms, of which about 318 ms is numeric factorisation, 204 ms symbolic analysis and 88 ms assembly, with model evaluation absent from the evaluation windows (`reports/l1_and_parallelism_2026-09-19.md`). In the decomposed solver the central block keeps at least nine tenths of that factorisation, since it keeps nine tenths of the nonzeros and factorisation cost grows faster than linearly in block size, while the satellite work per Jacobian update, automatic differentiation of the compiled models and dense factorisation of about 340 blocks of at most 72 unknowns, is bounded by roughly 50 ms. Amdahl's law then caps the parallel speedup of a Jacobian evaluation near 1.09, and the parallel share of an ordinary step is smaller still, since the generated model libraries are at most 2.4 % of the loop. This is the same ceiling the two OpenMP increments of 2026-09-20 measured from the other side.

The sequential mechanisms of the method are worth more than its parallelism on this system, and each has a cheaper counterpart in the existing fixed-step solver. First, a central block whose sparsity pattern is fixed, because the dense compiled-model blocks that are the suspected source of the pattern drift are removed from the sparse matrix and the solver owns the KLU calls, removes the symbolic analysis: 204 ms on 28 of 31 evaluations on the profiled operating point. The counterpart is a union-pattern cache in `SolverCommon::copySparseToKINSOL`, about fifty lines. Second, repeating the step instead of restoring after an event removes the second matrix set-up that every restoration performs, about 400 ms on the profiled evaluations, together with the restoration's own Newton iterations; the counterpart is a bypass of the restoration in `SolverCommonFixedTimeStep::reinit` with a restart at `hMin`. Third, refreshing satellite blocks every iteration while the central factorisation is retained across steps makes network refactorisations rarer; the counterpart is the untested `msbset` sweep already listed as the one cheap item left. On the 400 s benchmark the first mechanism alone is projected to take the two over-budget steps, 1083 ms and 1151 ms, under the budget; on the profiled operating point, whose worst step is 2766 ms and holds more than one evaluation, the three mechanisms together are projected to fall short, since the network numeric factorisation and the ordinary model evaluation of the step, together about 500 ms, are untouched by any of them.

The recommendation is therefore to run the three counterparts as experiments before any DDM code is written (section 13). The DDM retains one advantage that no counterpart reproduces: after an injector-local event, or when convergence degrades in one satellite, it refreshes that block alone and keeps the network factorisation. Whether that matters is decided by the `msbset` experiment: if the fixed-step solver keeps converging with a Jacobian retained for many steps, the advantage is worth little; if it loses convergence, and the failures coincide with injector mode changes, the DDM's selective refresh is the mechanism that restores it and the build is justified.

### 1.1 Outcome of the experiments, 2026-09-29

The three experiments of section 13 were run on 2026-09-28 and 2026-09-29 on the machine of record, serial, from one environment-gated binary, and the results are in the TRAISIM report `reports/phase0_experiments_2026-09-28.md`. They change the verdict above in three places.

The first mechanism, a fixed central pattern, buys nothing. A union-pattern cache in the fixed-step solver re-analysed the Euler matrix at every one of its evaluations on the 400 s benchmark and on the operating point, because every evaluation brings positions never seen before, and dropping those positions failed the run at step 10, because they are entries of magnitude one that switch with the state rather than derivatives drifting across the zero band. Where the union did hold, on the 4000 s scenario with 38 of 86 analyses spared, seconds over budget rose from 25 to 30: the analyses spared are the quiet ones, and every event second re-analyses in any case. The projection of section 1 that the symbolic analysis alone would clear the benchmark is refuted, and the central block's fixed pattern is no longer an argument for the solver. It also carries a cost the design must state: with a fixed pattern KLU refactors on a stale pivot order, and the cache applied to the restoration solver failed exactly that way.

The third mechanism, refreshing satellites while the central factorisation is retained, has no failure of the monolithic solver to fix. The refresh cadence sweep left the benchmark at exactly 12 evaluations at every `msbset`, because KINSOL resets its counter per call and the evaluations are forced by pattern-invariant events; the fixed-step solver already retains its Jacobian between events and can never retain it across one. The go/no-go rule of section 1 therefore answers no.

The second mechanism is the one that moved seconds under the budget, and it did so as a solver parameter rather than as a solver. Bypassing the algebraic restoration took the nine measured operating points from 82 to 63 seconds over budget and two of them into real time at a settled deviation of 1.0 × 10⁻⁴ pu, and lowered the 4000 s scenario's worst second from 3133 ms to 2441 ms while moving the cost into the seconds after each event, 25 against 24, at 1.9 × 10⁻³ pu. The restart at a small step that section 7 pairs with re-stepping was negative in the fixed-step solver on every case, because that solver's Jacobian carries the step size and each change of step costs an evaluation; the separate storage of the two Jacobian parts in section 5 is the feature that would spare this solver the same cost, and it is now a requirement the design must demonstrate rather than a convenience.

The build is not justified by the campaign. What the campaign leaves for the fixed-step solver is the bypass on the full set of operating points and on the four that fail inside the restoration, and a study of the post-event burst on the 4000 s scenario.

Both were done on 2026-09-29, sections 7 and 8 of the same report. On all 61 operating points the bypass takes the 57 that complete from 309 to 239 seconds over budget, puts 13 of them fully in real time where none was, and makes none worse; the four failing points fail identically with and without the restoration, in the fixed-step solver's step-reduction cascade after a line trip, with no restoration run before the failure, so the restoration is not where they fail. On the 4000 s scenario the post-event burst is Newton work common to both arms, six to ten iterations and a fresh Jacobian on the steps after an event whether or not the state was restored, so no restoration policy bounds it, and the second mechanism above would not remove it either. The bypass's own cost there is one failed step per run, and it is not a convergence failure: the second step after one event fails inside the residual evaluation, on an iterate the model cannot evaluate, and a doubled iteration limit changes nothing. That is the hazard the small post-event step of section 7 exists to cover, and the fixed-step measurement of its price stands. The verdict on the build is unchanged.

## 2. The system as Dynawo presents it to a solver

### 2.1 Layout of the global vectors

`ModelMulti::initBuffers` assigns each submodel a contiguous range of the global `y`, `yp`, `f`, `z` and `g` vectors in submodel order, through `SubModel::initSize` (`Modeler/Common/DYNSubModel.cpp:225-245`). The network model is added first by `Modeler::initSystem`, so it is submodel 0 with `yDeb = fDeb = 0`. The global residual vector is laid out as all submodel equations in order, then one row per connector equation, then one row `y = 0` per unconnected optional external variable (`DYNModelMulti.cpp:134-221`). A submodel's block is not square: its `EXTERNAL` variables occupy columns in its range but have no equation of its own.

### 2.2 Connectors add equations and never merge variables

Each variable keeps its own global index, and connecting two variables appends an equation. A continuous connection with n members contributes n − 1 rows of the form `y_ref − y_k = 0`; a flow connection contributes one row `Σ ±y_k = 0` (`DYNConnector.cpp:71-79, 488-601`). The Jacobian entries of these rows are ±1 and carry no `ẏ` term. A generator at a bus therefore appears in the Jacobian as follows: the generator's own rows reference only its own range, including its local copies of `V_re`, `V_im`, `i_re` and `i_im`; the bus's two balance rows reference the bus's own `ur`, `ui`, its neighbours' voltages, the switch currents of its voltage level and the bus's own `ir`, `ii` slots with coefficient −1 (`Models/CPP/ModelNetwork/DYNModelBusInjected.cpp:215-236`); and four connector rows tie the pair together, two continuous rows `bus.V − gen.V = 0` and two flow rows `bus.i + Σ gen.i = 0`.

Two further couplings exist. `DYNModelOmegaRef` receives every synchronous machine's speed through continuous connections and returns a reference speed per group through others; on the benchmark it has 955 variables and 620 equations. `ConnectorCalculatedVariable` submodels, 452 on the benchmark, each carry one equation `calc(y_source) − y = 0` whose entries reference the source model's variables, and their single variable is then connected to a consumer, typically an automaton, by an ordinary continuous connection.

### 2.3 Compiled Modelica blocks are dense

`ModelManager::evalJtAdept` runs one reverse-mode automatic-differentiation recording over the model's own `y` and `ẏ` and writes the full `sizeF × sizeY` block into the sparse matrix, one column per equation, dropping only the entries that `SparseMatrix::addTerm` judges zero (`DYNModelManager.cpp:320-383`). The zero test is `|v| ≤ precision/5`, which is 2 × 10⁻⁵ on the benchmark (`Common/DYNCommon.h:171`), so the pattern of these blocks is a function of the operating point. The sizes on the benchmark are 7 variables and 4 equations for `GeneratorPQInfiniteLimits`, 61 to 72 variables and 53 to 65 equations for the synchronous-machine models, 26 and 21 for `GeneratorPVRpcl2`, 31 and 28 for the static VAr compensator and 42 and 37 for the HVDC models; in every case the difference is the number of `EXTERNAL` slots.

### 2.4 The network block

`ModelNetwork` owns, per bus, `ur` and `ui` plus `ir` and `ii` when an injector is connected; per switch, a current pair with the equation `ur1 − ur2 = 0` when closed and `i = 0` when open; per restorative load, the two differential states `zP` and `zQ`; and per dangling line a fictitious node. Static lines, transformers, shunts, static VAr compensators and network generators own no variables and contribute currents and derivatives to their buses (`DYNModelLine.cpp:508-541`, `DYNModelBusInjected.cpp:606-656`). The benchmark network has 5,725 voltage levels, 7,075 lines, 2,138 two-winding transformers and 6 HVDC links. With `patternInvariantTopology` set, switch, line and transformer events keep the pattern fixed and are reported as `ALGEBRAIC_J_VALUES_MODE` (`DYNModelNetwork.cpp:1100-1148`).

### 2.5 Modes and events

`ModelMulti::evalMode` returns the maximum of the submodels' mode changes on the ordered enumeration `NO_MODE < DIFFERENTIAL_MODE < ALGEBRAIC_MODE < ALGEBRAIC_J_VALUES_MODE < ALGEBRAIC_J_UPDATE_MODE` (`Common/DYNEnumUtils.h:79-84`). A compiled model reports `ALGEBRAIC_J_UPDATE_MODE` whenever one of its discrete variables changes value, whatever the equations affected (`GeneratorPQ_Dyn.cpp:353-378` in the compiler's reference output), so an automaton limiter in one machine forces a full Jacobian rebuild in the monolithic solvers. `Solver::Impl::evalZMode` runs the `evalZ`, `evalG`, `evalMode` loop up to ten times and sets the `ModeChange` flag (`Solvers/Common/DYNSolverImpl.cpp:259-309`); the simulation loop then calls `reinit()` (`Simulation/DYNSimulation.cpp:1039-1123`).

### 2.6 Linear algebra in the existing solvers

The existing solvers reach KLU only through SUNDIALS. `SolverKINCommon::initCommon` creates a KINSOL memory, a compressed-sparse-row `SUNSparseMatrix` and a `SUNLinSol_KLU` linear solver, with `KINSetMaxSetupCalls(msbset)` governing how many nonlinear iterations pass between set-ups (`Solvers/AlgebraicSolvers/DYNSolverKINCommon.cpp:90-181`). `SolverCommon::copySparseToKINSOL` copies the Dynawo matrix and compares the row-index array against the previous one; a difference triggers `SUNLinSol_KLUReInit`, which repeats the symbolic analysis, and only that path runs `klu_analyze` (`Solvers/Common/DYNSolverCommon.cpp:33-82`). On the benchmark the nonzero count differs on every evaluation, so the analysis runs every time. The fixed-step solver reuses the factorisation across steps and forces a new one on the first step, after a divergence and after a root of type `ALGEBRAIC_J_UPDATE_MODE` or `ALGEBRAIC_J_VALUES_MODE` (`Solvers/FixedTimeStep/DYNSolverCommonFixedTimeStep.cpp:309-331, 400-413`). The third-party tree builds SUNDIALS without BLAS or LAPACK, and neither library nor Eigen is present in it.

### 2.7 Sizes on the cases this project runs

The Nordic case has 1,740 continuous variables, 950 discrete variables, 818 root functions and 20 synchronous machines with one `DYNModelOmegaRef`. The 400 s benchmark has 321,452 continuous variables, 336,412 discrete variables and 277,968 root functions, and takes 400 steps for 400 s with 13 Jacobian evaluations, 532 nonlinear iterations and 933 residual evaluations; every simulated second is one implicit Euler step of 1 s. An RTE operating point has 386,177 continuous variables, 398,629 discrete variables and 360,389 root functions.

## 3. Decomposition

### 3.1 Classification of submodels

The partition is decided once, at `init`, from the connector container and the variable and equation types, and never from the values. Every submodel falls into one of five classes.

The central block is `ModelNetwork` together with every submodel that the rules below cannot place elsewhere. A satellite injector is a submodel with `sizeF > 0` whose continuous connections lead only to one bus terminal of the network, through the `V` and `i` pairs of one `ACPIN`, and to explicit sources; a satellite twoport is the same with two terminals. An explicit coupler is a submodel with `sizeF > 0` whose continuous connections lead to satellites but not to network terminals, `DYNModelOmegaRef` being the instance on every case; its equations are solved once per step from the previous step's values of its inputs, and its outputs are held for the step (section 4.5). An explicit assignment is a submodel with `sizeF = 0`, whose continuous variables are all `EXTERNAL` copies closed by connector rows; the automata `DYNModelAcmc`, `CurrentLimitAutomatonWithIMaxFromInit`, `DYNModelSmacc`, `PhaseShifterI` and `DYNModelRST` are of this class, as is every `ConnectorCalculatedVariable`, whose one equation defines its one variable from the source. Their variables are assigned from their sources after the continuous solve, before the discrete phase.

The fallback rule places anything else in the central block, and the connector rows follow their variables: a continuous row tying a satellite's local voltage copy to its bus belongs to the satellite; a flow row summing satellite currents into a bus belongs to the central block; a row that references a central variable and a satellite variable in any other way makes that satellite central. The rule keeps the solver correct on any case at the cost of speed, and the census below shows that on the RTE cases it is never exercised.

### 3.2 Census on the benchmark

Of the 1,356 submodels of the diagnostic run, one is the network, 452 are calculated-variable connectors, 248 are `GeneratorPQInfiniteLimits`, about 335 are synchronous-machine models of eleven variants, 27 are `GeneratorPV` variants, 4 are static VAr compensators, 5 are HVDC models, one is the reference-speed model and the remaining 269 are automata and the secondary voltage regulation with `sizeF = 0`. The satellites therefore number about 620 with 26,885 variables between them, the twoports are the 5 HVDC links, and the central block is the network with its 294,902 variables plus the flow rows and calculated-variable rows.

### 3.3 Interface unknowns

Between a satellite and the central block the interface consists of two voltage components per terminal on the central side and two current components per terminal on the satellite side. A satellite's local unknowns are its own variables less the explicit inputs, and its local equations are its own residual rows plus the two continuous connector rows per terminal, which makes the local system square. The bus current slots `ir`, `ii` remain central unknowns defined by the flow rows, so the network's own pattern is not touched by the decomposition; the Schur terms land on the flow rows at the columns of the bus voltage, four new structural positions per connected bus and sixteen per twoport.

## 4. Formulation

### 4.1 Local and central systems

Let V denote the vector of central unknowns and x_i the local unknowns of satellite i, i = 1, ..., N. After algebraisation of the differential equations (section 5), the local system of satellite i is

    F_i(x_i, V) = [ f_i(x_i, u_i) ; P_i x_i − Q_i V ] = 0,                     (1)

where f_i is the satellite's own residual, u_i its explicit inputs held for the step, P_i the selector of its local voltage copies and Q_i the selector of the bus voltage in V. The central system is

    g(V) + Σ_i R_i x_i = 0,                                                    (2)

where g collects the network residuals, the calculated-variable rows and the flow rows with their central terms, and R_i is the ±1 selector that adds satellite i's local current copies into the flow rows of its bus.

### 4.2 Newton iteration and Schur reduction

One Newton iteration on (1) and (2) solves

    A_i Δx_i + B_i ΔV = −F_i,        A_i = ∂F_i/∂x_i,  B_i = [ 0 ; −Q_i ],   (3)
    D ΔV + Σ_i R_i Δx_i = −g,        D = ∂g/∂V.                               (4)

A_i is dense and small, B_i and R_i are trivial selectors. Eliminating Δx_i from (3),

    Δx_i = −A_i⁻¹ (F_i + B_i ΔV),                                             (5)

and substituting in (4) gives the reduced system

    ( D − Σ_i R_i A_i⁻¹ B_i ) ΔV = −g + Σ_i R_i A_i⁻¹ F_i.                     (6)

The Schur term R_i A_i⁻¹ B_i is a 2 × 2 block per terminal, or a 4 × 4 block across two terminals for a twoport, exactly as in [1, Fig. 4.2]. It is computed from the transposed solves T_i = R_i A_i⁻¹, two rows of length n_i, followed by the products T_i B_i and T_i F_i, which is how the reference implementation forms its `Ty`, `Tx`, `TB` and `tf` arrays. The matrix of (6) has the network's sparsity pattern plus the fixed set of Schur positions, so its symbolic analysis is performed once per topology.

### 4.3 Back-substitution and convergence

Once ΔV is known, (5) yields every Δx_i independently, in parallel. The residuals F_i and g are re-evaluated with the updated unknowns, and satellite i is declared converged when the scaled correction ‖Δx_i‖∞ falls below `blockTol` and its residual has not grown; the central block is converged when ‖g‖∞ falls below `netTol`. The step is converged when every block is.

### 4.4 The latent model

For a satellite with negligible dynamic activity the internal residual is neglected, F_i ≈ 0 in (5), which leaves the linear relation between its current and the bus voltage,

    I_i(t_n) = I_i(t*) + G_i(t*) ( V(t_n) − V(t*) ),   G_i = −T_i B_i,        (7)

where t* is the instant the satellite was declared latent. The Schur term of the latent model equals that of the full model, so switching a satellite between the two states leaves the reduced matrix unchanged [1, §4.6.3]. The switching criterion is the exponential moving standard deviation of the satellite's apparent power over a window `latencyWindow`, compared with `latencyTol`, with the return to the active state on an absolute deviation of the same size; `latencyTol = 0` disables the mechanism and recovers the exact solution.

### 4.5 Explicit couplers and the delayed reference frame

The reference speed couples every synchronous machine to every other, and in the monolithic Jacobian it contributes a dense row and column per group. Following [1, §1.2.6] and the reference implementation, the coupler's equations are solved once at the start of step n + 1 with the machine speeds of step n,

    ω_ref,n+1 = Φ( ω_n ),                                                      (8)

and the result is written into the machines' `EXTERNAL` slots for the whole step, which satisfies the corresponding connector rows exactly and removes them from (6). The solve of Φ reuses `SolverKINSubModel`, the per-submodel KINSOL wrapper that local initialisation already uses. The same one-step delay applies to every explicit input of a satellite whose source is not its own bus terminal, which on the RTE cases means the reference speed and the calculated variables consumed by automata. The delay is an approximation of the monolithic system and the results are not bit-identical to the fixed-step solver's; [1] reports that a slightly delayed centre-of-inertia frame is as good as the exact one in making the current and voltage components insensitive to the system frequency, and the regression gate for this solver is a settled deviation, not bit identity.

### 4.6 Equivalence with the monolithic Newton

With every block fresh and no explicit input, (3) to (6) are one Newton step on the monolithic system after a permutation of rows and columns, so the iteration inherits its convergence theory. Asynchronous block updates, skipping converged satellites and latency each turn it into a quasi-Newton or inexact Newton scheme whose convergence is analysed in [1, §4.7]: the first two leave the converged solution unchanged, the third does not.

## 5. Time integration

The integrator is a variable-step BDF of order one or two. With ω = h_n / h_(n−1), the second-order formula approximates the derivative at t_(n+1) as

    ẏ_(n+1) ≈ [ (1 + 2ω)/(1 + ω) y_(n+1) − (1 + ω) y_n + ω²/(1 + ω) y_(n−1) ] / h_n,   (9)

and the first-order formula, used on the first step and after every event, as (y_(n+1) − y_n) / h_n. Only variables typed `DIFFERENTIAL` carry a derivative; algebraic and external variables have ẏ = 0, which is what the fixed-step solver's `computeYP` does through `differentialVariablesIndices_`. Writing c for the leading coefficient of the formula divided by h_n, the algebraised local Jacobian is

    A_i = ∂f_i/∂x_i + c ∂f_i/∂ẋ_i,                                            (10)

and the two terms are stored separately, from `evalJt` with c = 0 and from `evalJtPrim`, so that a change of c re-forms A_i without a new differentiation. The central block depends on c only through the restorative loads and any differential-voltage bus, and is re-assembled on a change of c.

The step is controlled by effort, as in the reference implementation, and not by a local error estimate. After a step, the number of reduced-system solves it needed is compared with a target `effortTarget` and a deadband `effortDeadband`; below the band the step grows in proportion, above it the step shrinks, and the step is clipped to `hMin`, `hMax` and the next horizon. After an event the next step is `hMin` at order one, which is the mechanism that makes the restoration-free event handling of section 7 accurate. A change of step size re-forms every A_i and re-assembles and refactorises the central block, so the controller holds the step at `hMax` whenever the effort allows and changes it in as few distinct values as possible. On the benchmark `hMax` is set to the 1 s step of the reference solver, so that the real-time comparison is made at the same step where the system is quiescent.

## 6. The step

### 6.1 Blocks of one iteration

An iteration follows the four blocks of [1, Fig. 4.3]. Block A, in parallel over satellites and the central block: re-evaluate and refactorise the blocks flagged for update and compute their Schur terms and right-hand-side corrections. Block B, serial: assemble the corrections into (6), refactorise the reduced matrix if any Schur term or central entry changed, and solve for ΔV. Block C, in parallel over active, unconverged satellites: back-substitute (5), update x_i, re-evaluate F_i. Block D, in parallel: convergence tests. Latent satellites evaluate (7) in block C instead of solving.

### 6.2 Skipping converged blocks

A satellite that has converged is not solved in the following iterations of the step, but its residual is still evaluated and it re-enters the solve if the residual grows. A converged central block skips its solve in the same way. Both rules leave the converged solution unchanged.

### 6.3 Asynchronous updates

The A_i of a satellite is re-evaluated when its residual grows between iterations, when `injectorJacIterations` iterations have passed since its last update, or when its own discrete state changed. The central block is re-assembled and refactorised when the network mismatch grows, when `networkJacIterations` iterations have passed since its last factorisation, when its topology changed, or when c changed. Between those events the reduced matrix keeps the Schur terms it was factorised with, even if satellites have been refreshed since, which is the very dishonest Newton behaviour of the reference implementation: the refreshed satellite blocks act through the right-hand side and the back-substitution, the stale terms in the reduced matrix slow convergence, and the residual-growth rule triggers the refactorisation when the staleness bites.

### 6.4 Injector-local events: three routes

When a satellite reports a mode change and no central component does, only that satellite's A_i is re-evaluated and refactorised, at a cost below a millisecond. The reduced matrix is then handled by one of three routes, in increasing exactness and cost. The first keeps the factorisation and its stale Schur term, as in section 6.3, and lets the residual-growth rule decide; this is the reference implementation's behaviour and the default. The second corrects the retained factorisation by a rank-2 update: writing the change of the Schur block as U W Vᵀ with U and V the column selectors of the affected rows and columns and W the 2 × 2 difference,

    (D̃ + U W Vᵀ)⁻¹ = D̃⁻¹ − D̃⁻¹ U W ( I + Vᵀ D̃⁻¹ U W )⁻¹ Vᵀ D̃⁻¹,                (11)

so each event costs two triangular solves for D̃⁻¹ U, cached, and each subsequent iteration costs one extra dense product of size 2k, where k is the number of events since the last factorisation. The route is exact, needs no inverse of W, and is bounded by `maxRankUpdates`, beyond which the third route applies. The third route refactorises the reduced matrix, about 370 ms on the benchmark with the fixed pattern. Either way the whole Jacobian of the system is never rebuilt for an event confined to one injector, which is the property the request asks for.

### 6.5 Latency

Latency follows section 4.4, with the switching decision taken after each accepted step and never inside the iteration. The linearisation point is refreshed when a satellite returns to the active state, and every satellite returns to the active state after a network event or a step-size change, as the reference implementation does after a severe disturbance.

## 7. Discrete events: algebraic restoration against re-stepping

Dynawo and the reference algorithm handle a discrete change differently, and the choice decides both the cost of an event second and the semantics of the output. This section describes the two, states the evidence, and gives the reason the DDM follows the reference.

### 7.1 Dynawo's restoration

The fixed-step solver evaluates the root functions at the end of the step, updates the discrete variables and the modes, and returns with the `ModeChange` flag set; the step itself is not repeated (`DYNSolverCommonFixedTimeStep.cpp:333-352`). The simulation loop then calls `reinit()`, which for a mode change at or above `minimumModeChangeTypeForAlgebraicRestoration` runs `SolverKINAlgRestoration`: a KINSOL solve of the algebraic equations at the event instant, with the differential variables frozen, on a Jacobian obtained by evaluating the full matrix and erasing the differential rows and columns, on its own KLU symbolic and numeric factorisation, followed by an optional second solve for consistent derivatives, and repeated while the restored state changes further roots (`DYNSolverCommonFixedTimeStep.cpp:429-533`). An `ALGEBRAIC_J_UPDATE_MODE` event additionally forces a fresh factorisation of the Euler Jacobian at the next step. The restored algebraic values are written to the curves at the event instant, so an event appears as a vertical jump at t_n.

The measured cost is visible in the window profile of `operating_point_1`: two populations of Jacobian-evaluation windows, one of about 600 ms with 210 ms symbolic and 380 ms numeric, and one of about 410 ms with 255 ms symbolic and 140 ms numeric, consistent with the Euler matrix and the smaller restoration matrix respectively. An event second therefore carries the restoration matrix, the restoration's own iterations and the forced Euler rebuild, above 1 s before the step's ordinary work. The restoration also has a failure mode of its own: four RTE operating points terminate with `KINSOL fails to solve the problem` raised from the restoration, a few seconds after a scheduled line disconnection (`reports/future_work.md`, item 7), and the three KINSOL wrappers disagree on whether structural zeros survive into the factorisation (item 5). The corpus already sets the threshold to `ALGEBRAIC_J_UPDATE`, so restoration is skipped for the lesser modes; skipping it at the corpus step of 1 s is what produced the settled deviations of 1.85 × 10⁻² pu on `nordic` and 5.98 × 10⁻³ pu on `nordic_line_trip` between the configurations that run it and those that do not (`reports/future_work.md`, item 1).

### 7.2 The reference algorithm's re-stepping

The reference implementation detects the discrete changes after the continuous solve of the step, and if any discrete state changed it repeats the step from t_n with the new discrete state, up to a bounded number of cycles, after which the last limiter to act is locked (`simul_decomp.f90`, the `zchanged` and `max_cycles` logic). After a disturbance the next step is forced to `hmin` and the multistep history is reset to order one; only the blocks the event touched are refactorised, an injector's block for an injector event and the network block for a topology change. There is no separate solve at the event instant: the implicit step from (x_n, y_n) under the new equations computes algebraic values that satisfy the new constraints at t_n + h, and the inconsistent derivative that the differential states see for that one step is damped by the first-order formula, which is L-stable. With `hmin` of a few milliseconds the post-event values appear one small step after the event rather than at it, a difference below the resolution of phasor-scale curves. The cost of an event is one repeated Newton solve with retained factors, a few triangular solves and residual evaluations, plus the refactorisation of the touched blocks: below a millisecond for an injector, about 370 ms for the network with the fixed pattern.

### 7.3 Comparison

| | Restoration | Re-stepping |
|---|---|---|
| Work at an event | A second matrix, its analysis and factorisation, a KINSOL solve, then a forced rebuild of the step matrix | The step repeated with retained factors; only touched blocks refactorised |
| Injector-local event | Full-system restoration and full-system rebuild | One dense block, below 1 ms; network factors kept |
| Consistency at t_n | Algebraic values consistent at t_n⁺, written to curves | Consistent at t_n + h_min; curves show the jump one small step late |
| Derivatives | A second solve for consistent ẏ, needed by a variable-order integrator | Not needed: the order is reset to one |
| Failure mode | The algebraic solve may not converge; four operating points stop there | The repeated step may diverge; the step falls to h_min and the blocks are refreshed |
| Accuracy dependence | None on the step | On h_min after the event; at a large fixed step the jump is smeared, which is the measured 1.85 × 10⁻² pu case |

The two are equivalent in the limit of a vanishing post-event step and differ in cost by the whole second matrix. The DDM adopts re-stepping because its step control can restart at `hMin`, which the fixed-step solver cannot without a code change, because its satellites make the event-local refresh cheap, and because a restoration solve on the algebraic subsystem is exactly the kind of full-system operation the decomposition exists to avoid. The simulation loop's protocol is kept: the solver sets `ModeChange` after a step that changed the discrete state, so that the timeline and curves are updated, and its `reinit()` only resets the modes and marks the touched blocks for update. The price is the output semantic in the table's third row and a dependence on `hMin`; both are stated in the validation gates of section 12, and the counterpart experiment of section 13 measures the second in the fixed-step solver before the DDM commits to it.

## 8. Linear algebra and parallel regions

The central block is factorised with KLU called directly, not through SUNDIALS: `klu_analyze` once per pattern, `klu_refactor` on every value update, `klu_solve` and `klu_tsolve` per iteration. The matrix is kept in a persistent compressed-sparse-column structure whose pattern is the union of every pattern seen since the last analysis, with explicit zeros where the current evaluation dropped an entry; a new position outside the union triggers one re-analysis, and after warm-up none occurs until a topology change with `patternInvariantTopology` off. A zero pivot in `klu_refactor` falls back to `klu_factor` on the same analysis. This cache is written as a free-standing helper so that the counterpart experiment of section 13 can port it to the fixed-step solver unchanged.

Satellite blocks are dense, column-major, and factorised with partial pivoting; the reference kernels are LAPACK's `dgetrf` and `dgetrs`, with the transposed solves of section 4.2 as `dgetrs` with `TRANS = 'T'`. LAPACK is not in the third-party tree, and OpenBLAS is present on the development machine only as a system library, so the build adds `find_package(LAPACK)` behind an option, with a small in-tree partial-pivoting LU of the same interface as the fallback for blocks of at most a hundred unknowns; which of the two is the default is decision 2 of section 14.

Blocks A, C and D of section 6.1 are OpenMP parallel regions over satellites, under the existing `DYNAWO_USE_OPENMP` option, with a static schedule of chunk one and the thread count from the solver parameters. Each satellite writes only its own slices of `yLocal_`, `fLocal_` and its own dense storage, so no reduction is needed; the central block's `klu_common` is touched by the main thread only. One point needs verifying before the parallel regions are enabled: `evalJtAdept` creates its Adept stack per call, and Adept 2.1.1 keeps the active stack in a thread-local variable unless `ADEPT_STACK_THREAD_UNSAFE` was defined at its build; the third-party build flags must be checked and a unit test must run two compiled models' Jacobians concurrently.

## 9. Interface with the rest of Dynawo

The solver is `DYN::SolverDDM`, deriving from `Solver::Impl` exactly as `SolverIDA` does, in `Solvers/VariableTimeStep/SolverDDM/` with its own `CMakeLists.txt` modelled on `SolverIDA/CMakeLists.txt`, the `getFactory` and `deleteFactory` entry points, `desc_solver`, and a `test/` directory. It implements the members `Solver::Impl` leaves pure: `defineSpecificParameters`, `setSolverSpecificParameters`, `solverType`, `init`, `calculateIC`, `solveStep`, `reinit`, `setupNewAlgRestoration`, `getName`, `printSolveSpecific`, `printHeaderSpecific`, `setInitStep`, `updateStatistics` and `getTimeStep`, and overrides `computeYP`. `Solver::Impl::solve` already resets the state, reinitialises the modes and rotates the buffers once per call before `solveStep`, so the solver takes exactly one step per `solve` call and repeats it internally on a discrete change without rotating the buffers again. `calculateIC` reuses the existing KINSOL initialisation. `setupNewAlgRestoration` returns false and `reinit` performs no algebraic solve, as section 7 decides.

Two things outside the directory are touched. `VariableTimeStep/CMakeLists.txt` gains `add_subdirectory(SolverDDM)`. And `ModelMulti` gains two non-virtual, read-only accessors, one returning its submodel list and one returning its connector container, since both are private and no `Model` method exposes them; the solver obtains them through a `dynamic_cast` of the `Model` it is given, as `SolverIDA` obtains SUNDIALS-specific behaviour without the `Model` interface knowing. Non-virtual accessors leave the vtable layout unchanged, so no other library needs rebuilding. `ConnectorContainer` needs read access to each connector's members with their submodels and variable indices, which its present interface provides only to `ModelMulti`; one accessor there completes the set. Nothing in the fixed-step solvers, `SolverIDA`, the KINSOL wrappers or any model changes.

The per-step timing that the regression harness reads needs nothing from the solver: the simulation loop times every `solve` and `reinit` pair and writes `simRT.csv` itself (`DYNSimulation.cpp:1039-1123, 1634-1662`).

## 10. Parameters

In addition to the common parameters of `Solver::Impl`, the solver declares the following in its parameter set.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `hMin` | double | mandatory | Step after an event and lower bound of the step |
| `hMax` | double | mandatory | Upper bound of the step; the reference step on the benchmark |
| `hStart` | double | `hMin` | First step |
| `effortTarget` | double | 3 | Target number of reduced-system solves per step |
| `effortDeadband` | double | 1 | Half-width of the band around the target |
| `maxNewtonIter` | int | 20 | Iterations per step before the step is reduced |
| `netTol` | double | 1e-4 | Central residual tolerance |
| `blockTol` | double | 1e-4 | Satellite scaled-correction tolerance |
| `networkJacIterations` | int | 5 | Iterations between forced central refactorisations |
| `injectorJacIterations` | int | 3 | Iterations between forced satellite updates |
| `maxRankUpdates` | int | 0 | Rank-2 corrections before a refactorisation; 0 selects the first route of section 6.4 |
| `maxEventCycles` | int | 10 | Repetitions of a step on discrete changes before the last change is locked |
| `latencyTol` | double | 0 | Latency threshold in pu of apparent power; 0 disables |
| `latencyWindow` | double | 10 | Observation window of the moving statistics, s |
| `referenceFrameDelay` | bool | true | One-step delay of explicit couplers |
| `threads` | int | 1 | OpenMP threads for the satellite regions |

## 11. Statistics

`printEnd` reports, beside the common statistics, the number of steps attempted and accepted, the number of repeated steps on discrete changes, the reduced-system factorisations and solves, the satellite factorisations and solves, the pattern re-analyses, the rank-2 corrections applied, and the fraction of satellite-steps spent latent. The two dead counters of `Solver::Impl`, `nmeAlg_` and `nmeAlgJ_`, are not used.

## 12. Validation

The regression harness of the TRAISIM repository runs the cases and compares two configurations on status parity, step counts, curve identity, settled deviation and forced Jacobian rebuilds. Curves cannot be bit-identical to the fixed-step solver's by construction, since the delayed reference frame, the re-stepping and the variable step all change the arithmetic, so the gates for this solver are status parity on every case, a settled deviation below a stated tolerance against the fixed-step solver with restoration, and the real-time overlay of seconds over the one-second budget from `simRT.csv`. Because the solver's steps do not coincide with the reference's, the harness needs one extension: pairing of curve rows by interpolation at the reference timestamps, in addition to the exact pairing it has.

Bring-up is on Nordic, which has every class of submodel of section 3.1 at a size where a full Jacobian can be dumped and compared. The first artefact is not the solver but a debug pass alongside the fixed-step solver, which builds the partition, extracts the blocks and asserts that the block-bordered assembly equals the global matrix up to permutation, entry by entry. The second is the serial solver on Nordic, then the 400 s benchmark, then the 4000 s scenario and the 57 operating points on the machine of record, serial, judged first by seconds over budget and second by wall clock. Latency is measured last and separately, since it is the one mechanism that changes the converged solution.

## 13. Scope, phases and the go/no-go experiments

Before any solver code, three experiments measure the sequential mechanisms of section 1 in the fixed-step solver, each on a throwaway branch and each judged with the harness on the machine of record.

1. The union-pattern cache in `SolverCommon::copySparseToKINSOL`, about fifty lines, with an explicit-zero fill and a re-analysis only on a new position. Success is the symbolic analysis disappearing from the steady-state evaluations and the 400 s benchmark at 0 of 400 seconds over budget.
2. The `msbset` sweep at 10, 20, 50 and 100 with the present `maxNewtonTry`, recording evaluations, over-budget seconds and convergence failures, and for every failure whether an injector mode change preceded it. Success is fewer evaluations without failures; the failure pattern decides whether the DDM's selective refresh has a job.
3. A bypass of the restoration in `SolverCommonFixedTimeStep::reinit` combined with a step reduction to `hMin` on `ModeChange` and the existing growth back, measured on the 4000 s scenario and the operating points for over-budget seconds and settled deviation against the run with restoration. Success is a settled deviation well below the 1.85 × 10⁻² pu that skipping without a restart produced.

The experiments were run on 2026-09-28 and 2026-09-29; section 1.1 records the outcome, which does not justify the build. The phases below stand as the plan if that changes. If experiments 1 and 2 leave the benchmark and the operating points short of real time, the DDM is built in five phases: the partition and block extraction as the debug pass of section 12; the serial solver with BDF, the Newton scheme, KLU on the central block and dense satellites; re-stepping and explicit couplers; OpenMP, latency and the rank-2 route; and the campaign on the RTE cases. The estimate is 3,000 to 4,000 lines including tests, on the same order as `SolverIDA` plus its share of the fixed-step scaffolding, and the harness extension of section 12 comes on top. If the experiments bring the cases into real time on their own, the DDM remains a platform for modularity and latency rather than a route to the goal, and its build is a separate decision.

## 14. Open decisions

1. Whether the three experiments of section 13 run before any solver code, as recommended.
2. The dense kernel: a LAPACK dependency behind a build option, or the in-tree LU as the default with LAPACK optional.
3. Whether calculated-variable inputs to satellites, if any case has them, are delayed one step or coupled exactly through the chain rule.
4. Whether the output semantic of section 7.3, post-event values one `hMin` after the event, is acceptable for the curves the project compares.
5. Whether the rank-2 route of section 6.4 is built in the first version or left behind `maxRankUpdates = 0`.
6. The tolerance of the settled-deviation gate against the fixed-step solver.

## 15. References

1. P. Aristidou, "Time-domain simulation of large electric power systems using domain-decomposition and parallel processing methods," PhD thesis, University of Liège, 2015, chapter 4 and section 1.2.6.
2. P. Aristidou, D. Fabozzi and T. Van Cutsem, "Dynamic simulation of large-scale power systems using a parallel Schur-complement-based decomposition method," IEEE Transactions on Parallel and Distributed Systems, vol. 25, no. 10, 2014.
3. D. Fabozzi and T. Van Cutsem, "Simplified time-domain simulation of detailed long-term dynamic models," IEEE PES General Meeting, 2009, for the delayed centre-of-inertia frame.
4. T. A. Davis and E. Palamadai Natarajan, "Algorithm 907: KLU, a direct sparse solver for circuit simulation problems," ACM Transactions on Mathematical Software, vol. 37, no. 3, 2010.
5. The reference implementation, `simul_decomp.f90` of RAMSES: `comp_factor_Jac` for the block factorisation and Schur terms, `solve_injectors` for the satellite solve and the latent model, `update_latency` for the switching criterion, and the main loop for the effort-based step control and the re-stepping on discrete changes.
