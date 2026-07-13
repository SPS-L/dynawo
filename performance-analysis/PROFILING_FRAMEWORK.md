# Dynaωo Performance Profiling Framework

> **Branch:** `3_performance-analysis-framework`
> **Scope:** `DYNSolverProfiler` — build-time instrumentation framework spanning the Simulation, Solver, and KLU linear-algebra layers

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Build-Time Activation](#2-build-time-activation)
3. [Phase Taxonomy](#3-phase-taxonomy)
4. [Instrumentation Points](#4-instrumentation-points)
5. [Output Formats](#5-output-formats)
6. [Real-Time Tracking (simRT.csv)](#6-real-time-tracking-simrtcsv)
7. [Per-Commit Changelog](#7-per-commit-changelog)
8. [Audit — Known Issues and Limitations](#8-audit--known-issues-and-limitations)
9. [Usage Cookbook](#9-usage-cookbook)

---

## 1. Architecture Overview

The profiling framework is a lightweight, zero-overhead-when-disabled system built entirely on RAII and preprocessor guards.

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Simulation layer                              │
│  DYNSimulation.cpp                                                   │
│  ├─ DYN_PROFILE_PHASE_MEM(PHASE_SIMULATION_LOOP)  [outer scope]      │
│  ├─ DYN_PROFILE_RECORD_TIMESTEP(t, ms, KB)        [per step]         │
│  └─ DYN_PROFILE_PHASE(PHASE_CURVES_UPDATE)        [updateCurves()]   │
│  └─ DYN_PROFILE_PHASE(PHASE_IO)                   [terminate()]      │
├──────────────────────────────────────────────────────────────────────┤
│                        Solver layer                                  │
│  DYNSolverIDA.cpp                                                    │
│  ├─ DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC)  [SolverIDA::calculateIC()]  │
│  ├─ DYN_PROFILE_PHASE(PHASE_RESIDUAL_EVAL)     [evalF() callback]    │
│  ├─ DYN_PROFILE_PHASE(PHASE_JACOBIAN_EVAL)     [evalJ() callback]    │
│  ├─ DYN_PROFILE_PHASE(PHASE_ROOT_EVAL)         [evalG() callback]    │
│  └─ DYN_PROFILE_PHASE(PHASE_SOLVER_STEP)       [solveStep()]         │
│  DYNSolverSIM.cpp / DYNSolverTRAP.cpp                                │
│  ├─ DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC)  [calculateIC()]       │
│  DYNSolverCommonFixedTimeStep.cpp                                    │
│  ├─ DYN_PROFILE_PHASE(PHASE_ROOT_EVAL)         [calculateICCommon()] │
│  ├─ DYN_PROFILE_PHASE(PHASE_ROOT_EVAL)         [updateZAndMode()]    │
│  ├─ DYN_PROFILE_PHASE(PHASE_ROOT_EVAL)         [reinit() ×2]         │
│  ├─ DYN_PROFILE_PHASE(PHASE_NR_SOLVE)          [callAlgebraicSolver()]│
│  └─ DYN_PROFILE_PHASE(PHASE_SOLVER_STEP)       [solveStepCommon()]   │
│  DYNSolverCommon.cpp                                                 │
│  ├─ DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE)                            │
│  ├─ DYN_PROFILE_PHASE(PHASE_MODE_EVAL)                               │
│  ├─ DYN_PROFILE_PHASE(PHASE_DISCRETE_EVAL)                           │
│  ├─ DYN_PROFILE_PHASE(PHASE_MATRIX_COPY)                             │
│  ├─ DYN_PROFILE_PHASE(PHASE_KINSOL_SOLVE)                            │
│  └─ DYN_PROFILE_PHASE(PHASE_REINIT)                                  │
├──────────────────────────────────────────────────────────────────────┤
│                        KLU linear-algebra layer                      │
│  DYNSolverKINEULERCommon.cpp / DYNSolverIDA* (ops-patching)          │
│  ├─ DYN_PROFILE_PHASE(PHASE_KLU_SYMBOLIC)  [klu_analyze]             │
│  └─ DYN_PROFILE_PHASE(PHASE_KLU_SETUP)     [SUNLinSolSetup]          │
├──────────────────────────────────────────────────────────────────────┤
│                        Profiler singleton                            │
│  DYNSolverProfiler.h / DYNSolverProfiler.cpp                         │
│  ├─ SolverProfiler (singleton, PHASE_COUNT=17 buckets)               │
│  ├─ PhaseTimer (RAII wrapper)                                        │
│  └─ Auto-export via DYNAWO_PROFILE_OUTPUT env-var on destructor      │
└──────────────────────────────────────────────────────────────────────┘
```

### Design Principles

| Principle | Implementation |
|---|---|
| **Zero overhead when disabled** | All macros expand to `((void)0)` unless `-DDYNAWO_PROFILING` |
| **No instrumentation in hot path** | `PhaseTimer` uses `std::chrono::high_resolution_clock` — one `now()` per constructor and one per destructor |
| **Separation of concerns** | Profiler does not know about solver internals; it only receives `(phase, elapsed, mem)` tuples |
| **RAII lifetime** | Timers automatically record on scope exit — no manual start/stop needed |
| **Auto-export** | `SolverProfiler::~SolverProfiler()` checks `DYNAWO_PROFILE_OUTPUT` and writes CSV or JSON |

---

## 2. Build-Time Activation

### CMake Option

```bash
cmake -DDYNAWO_PROFILING=ON ..
```

This defines the `DYNAWO_PROFILING` preprocessor symbol, which activates all `DYN_PROFILE_*` macros and sets `SolverProfiler::enabled_ = true` in the constructor.

**Standard (non-profiling) build:**
```bash
cmake -DDYNAWO_PROFILING=OFF ..
# or simply omit — OFF is the default
```

All macro call sites compile to `((void)0)`, generating no code. There is no runtime branch on `enabled_`.

### Environment Variables (runtime)

| Variable | Effect |
|---|---|
| `DYNAWO_PROFILE_OUTPUT=<path>.csv` | Auto-export phase summary + timestep time-series as CSV on simulation exit |
| `DYNAWO_PROFILE_OUTPUT=<path>.json` | Auto-export as JSON (detected by `.json` suffix) |

The export is triggered unconditionally in `SolverProfiler::~SolverProfiler()` whenever `PHASE_SIMULATION_LOOP` has been entered at least once, so the path detection is reliable even if the simulation exits via an exception.

---

## 3. Phase Taxonomy

Phases are defined in `ProfilePhase` enum (`DYNSolverProfiler.h`). There are currently **17 active phases** plus the sentinel `PHASE_COUNT`.

### Phase Hierarchy and Parent–Child Relationships

Phases are **inclusive** (a parent's `totalTime` includes all nested children). The `printReport()` function computes **exclusive** times by subtracting direct children.

```
SIMULATION_LOOP  (memory-tracked, RAII scope in simulate())
├── CALCULATE_IC       (initial condition solve — called before the main loop)
│   └── ROOT_EVAL      (zero-crossing eval inside calculateICCommon / SolverIDA::calculateIC)
├── SOLVER_SOLVE       (per solver_->solve() call)
│   ├── SOLVER_STEP        (per IDA/Euler integration step)
│   │   ├── RESIDUAL_EVAL      (F(x,x',y,t) evaluation)
│   │   ├── JACOBIAN_EVAL      (∂F/∂x + ∂F/∂ẋ assembly)
│   │   │   ├── MATRIX_COPY        (sparse matrix copy before factorization)
│   │   │   ├── KLU_SYMBOLIC       (klu_analyze — structure-changing steps)
│   │   │   └── KLU_SETUP          (SUNLinSolSetup / klu_refactor)
│   │   └── ROOT_EVAL          (zero-crossing detection — also fires in updateZAndMode / reinit)
│   ├── NR_SOLVE           (algebraic Newton-Raphson, fixed-step solver)
│   │   └── KINSOL_SOLVE       (KINSOL inner solve)
│   │       ├── RESIDUAL_EVAL  (shared with above — see W-SHARED-PHASE-DOUBLE-COUNT)
│   │       └── JACOBIAN_EVAL  (shared)
│   ├── DISCRETE_EVAL      (z-variable / event-trigger evaluation)
│   ├── MODE_EVAL          (mode detection and state machine update)
│   └── REINIT             (solver reinit after mode change)
├── CURVES_UPDATE      (updateCurves() — calculated-var flush)
└── IO                 (terminate() — file export, final state dump)
```

> **Note on shared children**: `RESIDUAL_EVAL`, `JACOBIAN_EVAL`, and `ROOT_EVAL` appear at multiple call sites and under multiple logical parents. Exclusive-time reporting uses runtime parent-child attribution (not global child totals), so shared-phase time is subtracted only from the parent scope where it was observed.

### Phase Reference Table

| Enum | String | Source | Triggered by |
|---|---|---|---|
| `PHASE_SIMULATION_LOOP` | `SimulationLoop` | `DYNSimulation.cpp` | Outer `simulate()` loop — memory-tracked |
| `PHASE_SOLVER_SOLVE` | `SolverSolve` | `DYNSolverCommon.cpp` | `solveStepCommon()` entry |
| `PHASE_CALCULATE_IC` | `CalculateIC` | `DYNSolverIDA.cpp` (IDA) · `DYNSolverSIM.cpp` / `DYNSolverTRAP.cpp` (SIM/TRAP) | `calculateIC()` overrides only — `calculateICCommon()` deliberately carries no timer (it would self-nest and double-count) |
| `PHASE_SOLVER_STEP` | `SolverStep` | `DYNSolverIDA.cpp` · `DYNSolverCommonFixedTimeStep.cpp` | Per IDA/Euler step |
| `PHASE_JACOBIAN_EVAL` | `JacobianEval` | `DYNSolverIDA.cpp` (`evalJ` callback) | Jacobian callback |
| `PHASE_RESIDUAL_EVAL` | `ResidualEval` | `DYNSolverIDA.cpp` (`evalF` callback) | Residual callback |
| `PHASE_ROOT_EVAL` | `RootEval` | `DYNSolverIDA.cpp` (`evalG`) · `DYNSolverCommonFixedTimeStep.cpp` (×3) | Zero-crossing callback + fixed-step evalG calls |
| `PHASE_MODE_EVAL` | `ModeEval` | `DYNSolverCommon.cpp` | Mode detection |
| `PHASE_DISCRETE_EVAL` | `DiscreteEval` | `DYNSolverCommon.cpp` | Discrete variable update |
| `PHASE_NR_SOLVE` | `NRSolve` | `DYNSolverCommonFixedTimeStep.cpp` | Fixed-step algebraic solve |
| `PHASE_MATRIX_COPY` | `MatrixCopy` | `DYNSolverCommon.cpp` | Pre-factorization matrix copy |
| `PHASE_KINSOL_SOLVE` | `KINSOLSolve` | `DYNSolverCommon.cpp` | KINSOL inner solve |
| `PHASE_REINIT` | `Reinit` | `DYNSolverCommon.cpp` | Post-mode-change solver reinit |
| `PHASE_IO` | `IO` | `DYNSimulation.cpp` | `Simulation::terminate()` |
| `PHASE_KLU_SYMBOLIC` | `KLUSymbolic` | KLU ops-patch (`DYNSolverCommon.cpp`) | `klu_analyze` (new sparsity structure) |
| `PHASE_KLU_SETUP` | `KLUSetup` | KLU ops-patch (`DYNSolverCommon.cpp`) | `SUNLinSolSetup` → `klu_refactor` |
| `PHASE_CURVES_UPDATE` | `CurvesUpdate` | `DYNSimulation.cpp` | `updateCurves()` after early-return guard |

---

## 4. Instrumentation Points

### 4.1 Simulation Layer (`DYNSimulation.cpp`)

**`Simulation::simulate()`**

```cpp
// Outer loop — memory-tracked
DYN_PROFILE_PHASE_MEM(PHASE_SIMULATION_LOOP);   // RAII, fires at '}' of inner scope

// Per-timestep timing — stepStartTime declared unconditionally so both
// real-time tracking and the profiler can share it without ordering dependency.
auto stepStartTime = std::chrono::high_resolution_clock::now();
// ... solver_->solve() and all step work ...

// Real-time tracking block (compiled unconditionally, gated at runtime)
if (enableRealTimeTracking_) {
  auto stepEndTime = std::chrono::high_resolution_clock::now();
  double stepTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
      stepEndTime - stepStartTime).count() / 1000.0;
  auto accDur = std::chrono::duration_cast<std::chrono::microseconds>(
      stepEndTime - simulationStartTime_);
  double accTimeS = accDur.count() / 1000000.0;
  timingData_.emplace_back(tCurrent_, stepTimeMs, accTimeS);
}

// Profiler timestep record — independent of enableRealTimeTracking_
#ifdef DYNAWO_PROFILING
{
  auto stepEndTime = std::chrono::high_resolution_clock::now();
  double stepTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
      stepEndTime - stepStartTime).count() / 1000.0;
  DYN_PROFILE_RECORD_TIMESTEP(tCurrent_, stepTimeMs,
                              DYN::SolverProfiler::getCurrentMemoryKB());
}
#endif
```

> **Note**: `stepEndTime` is captured separately inside each feature's block (see §8 B-STEPTIME-DOUBLE). Both blocks are independent; when both features are active, `chrono::now()` is called twice per step.

**`Simulation::updateCurves()`**

```cpp
void Simulation::updateCurves(const bool updateCalculatedVariable) const {
  if (exportCurvesMode_ == EXPORT_CURVES_NONE &&
      exportFinalStateValuesMode_ == EXPORT_FINAL_STATE_VALUES_NONE)
    return;               // ← zero cost if no curves configured
  DYN_PROFILE_PHASE(PHASE_CURVES_UPDATE);  // ← placed after early-return guard
  // ...
}
```

**`Simulation::terminate()`**

```cpp
void Simulation::terminate() {
  DYN_PROFILE_PHASE(PHASE_IO);   // covers all file export operations
  // ...
}
```

### 4.2 Solver Layer

#### `DYNSolverIDA.cpp` — IDA (variable-step BDF) solver

`PHASE_CALCULATE_IC` is instrumented **directly in `SolverIDA::calculateIC()`**:

```cpp
// DYNSolverIDA.cpp
void SolverIDA::calculateIC(const double /*tEnd*/) {
  DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC);   // ← IDA call site
  // ... KINSOL algebraic restoration + IDACalcIC loop ...
}
```

The inner IDA callbacks are also instrumented in this file:

```cpp
int SolverIDA::evalF(...) {
  DYN_PROFILE_PHASE(PHASE_RESIDUAL_EVAL);
}

int SolverIDA::evalJ(...) {
  DYN_PROFILE_PHASE(PHASE_JACOBIAN_EVAL);
}

int SolverIDA::evalG(...) {
  DYN_PROFILE_PHASE(PHASE_ROOT_EVAL);
}

void SolverIDA::solveStep(...) {
  DYN_PROFILE_PHASE(PHASE_SOLVER_STEP);
}
```

#### `DYNSolverCommonFixedTimeStep.cpp` — fixed-step solver (SolverSIM / SolverTRAP)

`PHASE_CALCULATE_IC` is **also instrumented** in the fixed-step common base, so both DynaSwing (IDA) and DynaWaltz (SIM/TRAP) runs correctly record IC-solve time:

```cpp
// DYNSolverCommonFixedTimeStep.cpp
void SolverCommonFixedTimeStep::calculateICCommon() {
  DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC);  // ← fixed-step call site
  // ...
  {
    DYN_PROFILE_PHASE(PHASE_ROOT_EVAL);        // ← ROOT_EVAL inside IC
    model_->evalG(tSolve_, g0_);
  }
  // ...
}
```

`PHASE_ROOT_EVAL` fires at **three additional sites** within the fixed-step solver (inside `updateZAndMode()` and `reinit()` ×2), and `PHASE_NR_SOLVE` wraps the inner algebraic solve:

```cpp
int SolverCommonFixedTimeStep::callAlgebraicSolver() {
  DYN_PROFILE_PHASE(PHASE_NR_SOLVE);
  // ...
}

void SolverCommonFixedTimeStep::solveStepCommon(...) {
  DYN_PROFILE_PHASE(PHASE_SOLVER_STEP);
  // ...
}
```

> **Note**: `PHASE_ROOT_EVAL` appears under multiple parents in fixed-step paths (`CALCULATE_IC`, `SOLVER_STEP`, `REINIT`). This is handled by contextual parent-child attribution in the profiler.

#### `DYNSolverCommon.cpp` — shared solver base

The common solver base class instruments phases that apply to both IDA and fixed-step paths:

```cpp
void SolverCommon::solveStepCommon(...) {
  DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE);
  // ...delegates to IDA or Euler step...
}
```

Matrix operations and KLU patching are also performed here (see §4.3).

### 4.3 KLU Layer (ops-patching)

KLU timing is the most architecturally interesting part. Because KLU is called through SUNDIALS `SUNLinearSolver` ops pointers, direct wrapping of KLU C functions would intercept all linear solvers globally. Instead, a **per-instance ops-patch** is applied:

```cpp
// During solver initialization (after SUNLinSol_KLU creation):
origSetup_ = LS->ops->setup;    // save original
LS->ops->setup = profiledSetup; // replace with wrapper

// Wrapper:
static int profiledSetup(SUNLinearSolver LS, SUNMatrix A) {
  DYN_PROFILE_PHASE(PHASE_KLU_SETUP);
  return captured_original_setup(LS, A);
}
```

`PHASE_KLU_SYMBOLIC` (`klu_analyze`) is triggered separately when the Jacobian sparsity structure changes — typically only on the first step or after a topology change.

**Renamed in commit `d8f9b6e`**: `KLUFactor` → `KLUSetup` to reflect that the patched function is `SUNLinSolSetup` (which calls `klu_refactor`), not the lower-level `klu_factor`.

---

## 5. Output Formats

### 5.1 `printReport()` — Dynaωo trace log

Called explicitly via `DYN_PROFILE_PRINT_REPORT()` at the end of `simulate()`. Writes to the Dynaωo info log.

**Inclusive table** — columns: `Phase | Total(s) | Calls | Avg(ms) | Min(ms) | Max(ms) | Pct(%)`

**Exclusive breakdown** — shows `Phase | Excl(s) | Pct(%)` for the six phases that have instrumented children: `SimulationLoop`, `SolverSolve`, `SolverStep`, `JacobianEval`, `NRSolve`, `KINSOLSolve`.

**Timestep count** and **Peak RSS** are appended at the bottom. Peak RSS is derived from the timestep series (`timestepRecords_`) when available; if no timestep records exist (e.g., `DYN_PROFILE_RECORD_TIMESTEP` was never called), it falls back to scanning `PhaseStats::peakMemoryKB` across all phases.

### 5.2 CSV Export (`DYNAWO_PROFILE_OUTPUT=<file>.csv`)

Two sections in a single file, separated by a blank line and identified by `# PHASES` / `# TIMESTEPS` comment headers:

**Section 1 — Phase summary:**
```
# PHASES
phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb
SimulationLoop,12.345678,1,12345.6780,12345.6780,12345.6780,204800
SolverSolve,11.234567,1000,11.2346,...
...
```

**Section 2 — Timestep time-series (appended after a blank line):**
```

# TIMESTEPS
sim_time,step_duration_ms,memory_kb
0.000000,1.2340,204800
0.010000,1.1890,204816
...
```

The `# PHASES` and `# TIMESTEPS` comment markers allow standard CSV parsers (pandas, Excel) to detect section boundaries. In pandas, skip comment lines with `pd.read_csv(f, comment='#')` after splitting the file at the blank line.

### 5.3 JSON Export (`DYNAWO_PROFILE_OUTPUT=<file>.json`)

```json
{
  "phases": [
    {"name": "SimulationLoop", "total_seconds": 12.345678,
     "call_count": 1, "avg_ms": 12345.678, "min_ms": 12345.678,
     "max_ms": 12345.678, "peak_memory_kb": 204800},
    ...
  ],
  "timesteps": [
    {"sim_time": 0.0, "step_duration_ms": 1.234, "memory_kb": 204800},
    ...
  ]
}
```

### 5.4 `analyze_profile.py`

Python post-processor that reads the CSV export and generates:
- `time_breakdown_pie.png` — phase time shares (pie)
- `hotspot_bar.png` — exclusive time bar chart
- `step_duration_ts.png` — step duration time-series
- `memory_usage_ts.png` — memory growth over simulation time

Usage:
```bash
export DYNAWO_PROFILE_OUTPUT=profile.csv
./myDynawoSimulation
python3 performance-analysis/analyze_profile.py profile.csv
```

---

## 6. Real-Time Tracking (`simRT.csv`)

This is a **separate, independent feature** from the profiler. It is controlled by `<simulation enableRealTimeTracking="true"/>` in the JOB XML file.

| Feature | `SolverProfiler` (`DYNAWO_PROFILING`) | `simRT.csv` (`enableRealTimeTracking`) |
|---|---|---|
| Activation | CMake build flag | JOB XML attribute |
| Overhead | Zero when disabled | `chrono::now()` per step always compiled |
| Timestep data | `sim_time, step_ms, memory_kb` | `simulation_time, computation_time_ms, accumulated_computation_time_s` |
| Output | `DYNAWO_PROFILE_OUTPUT` env-var | `<outputsDir>/simRT.csv` |
| Phase data | Yes (17 phases) | No |

**Important (post B4)**: `DYN_PROFILE_RECORD_TIMESTEP` is executed in its own `#ifdef DYNAWO_PROFILING` scope, independently of `enableRealTimeTracking_`. Both features share the same `stepStartTime` variable (declared unconditionally before `solver_->solve()`). The `stepEndTime` variable is computed separately inside each feature's block — see §8 B-STEPTIME-DOUBLE for the minor overhead implication.

---

## 7. Per-Commit Changelog

### Commit 1 — `1dce6fd` — *KLU profiling: integrate KLU numeric and symbolic factorization timing*

Added `PHASE_KLU_SYMBOLIC` and initial `PHASE_KLU_FACTOR` (later renamed) to the `ProfilePhase` enum. Implemented the per-instance ops-patch mechanism to intercept `SUNLinSolSetup` without modifying SUNDIALS source. Added instrumentation to the KLU setup call-path in `DYNSolverKINEULERCommon.cpp` and/or the IDA linear solver wrapper.

### Commit 2 — `f1a82f8` — *docs: add perf record / KLU analyze sub-symbol profiling section to README*

Extended `performance-analysis/README.md` with a new section documenting how to use `perf record` + `perf report` to drill below KLU phase boundaries into individual KLU C functions (`klu_analyze`, `klu_refactor`, `klu_solve`). Also documents the sub-symbol profiling methodology using `perf annotate`.

### Commit 3 — `d8f9b6e` — *fix: harden KLU profiler — per-instance capture, assert, rename KLUFactor→KLUSetup*

Three changes:
1. **Per-instance capture**: Changed from a static global function pointer to a per-instance capture stored alongside the ops-patching, preventing interference if multiple linear solver instances are created.
2. **Assert**: Added `assert(origSetup_ != nullptr)` to catch ops-patching failures at debug time.
3. **Rename**: `PHASE_KLU_FACTOR` → `PHASE_KLU_SETUP` and `"KLUFactor"` → `"KLUSetup"` in `phaseToString()` to accurately reflect that the intercepted function is `SUNLinSolSetup` (which internally calls `klu_refactor` or `klu_factor` depending on whether symbolic analysis was already done).

### Commit 4 — `04f5237` — *fix(profiler): add PHASE_CURVES_UPDATE to ProfilePhase enum (B3a)*

Added `PHASE_CURVES_UPDATE` as enum value 16 in `ProfilePhase`, with a Doxygen comment. Updated `phaseToString()` to return `"CurvesUpdate"`. Updated `PHASE_COUNT` (sentinel value) accordingly.

### Commit 5 — `898df35` — *fix(profiler): add CurvesUpdate string + inclusive-time note + exclusive breakdown in printReport() (B3a, B5)*

1. Added `"CurvesUpdate"` to `phaseToString()` switch (guard for the `default` path).
2. Added the `NOTE: times are INCLUSIVE` line to the `printReport()` table header.
3. Implemented the **exclusive-time breakdown** section in `printReport()` using the `excl()` lambda with hard-coded parent→children relationships.
4. Added `PHASE_CURVES_UPDATE` as a child of `PHASE_SIMULATION_LOOP` in the exclusive calculation for `SimulationLoop`.

### Commit 6 — `f1b8800` — *fix(profiler): instrument solveStepCommon (B1) and calculateICCommon (B2) with profiler phases*

Added `DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE)` at the entry of `SolverCommon::solveStepCommon()`. Also added `DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC)` at the entry of `SolverIDA::calculateIC()` — the IDA-specific initial-condition solve entry point. These are the two most important coarse-grained instrumentation points for separating initialization cost from simulation loop cost.

### Commit 7 — `4f7340a` — *profiler: fix B3b + B4 — instrument updateCurves() and decouple RECORD_TIMESTEP from enableRealTimeTracking_*

1. **B3b**: Added `DYN_PROFILE_PHASE(PHASE_CURVES_UPDATE)` inside `Simulation::updateCurves()` after the early-return guard. Closes the gap between the enum definition (B3a) and actual instrumentation.
2. **B4**: Moved `DYN_PROFILE_RECORD_TIMESTEP` out of the `if (enableRealTimeTracking_)` block into a standalone `#ifdef DYNAWO_PROFILING` scope, so profiling builds always record per-timestep entries regardless of JOB XML settings.

### Commit 8 — `bf4376b` — *fix(profiler): add CALCULATE_IC to SimulationLoop exclusive child list (B-EXCL-CALC_IC)*

`PHASE_CALCULATE_IC` runs inside the `PHASE_SIMULATION_LOOP` RAII scope (via `Simulation::init()` → `SolverIDA::calculateIC()`) but was absent from the `excl()` child list for `SimulationLoop` in `printReport()`. This caused IC-solve time to be misattributed as `SimulationLoop` dispatch overhead in the exclusive-time breakdown, overstating `SimulationLoop` exclusive time for stiff cases with a significant t=0 KINSOL solve.

Two changes in `DYNSolverProfiler.cpp`:
1. Added `PHASE_CALCULATE_IC` to the `exclSimLoop` child initialiser list.
2. Updated the parent→children comment block above the `excl()` lambda to include `CalculateIC` under `SimulationLoop`.

Closes audit item **B-EXCL-CALC_IC**.

### Commit 9 — `4789f34` — *docs(profiling): audit pass — fix CALCULATE_IC call-site docs, add W-CALC_IC-IDA-ONLY and W-SHARED-PHASE warnings*

Audit pass performed 2026-04-04 against branch HEAD. Findings:

1. **Architecture Overview (§1)**: Corrected to show `PHASE_CALCULATE_IC` in `DYNSolverIDA.cpp` (actual instrumentation location, confirmed by code inspection of `DYNSolverIDA::calculateIC()`). Previous diagram incorrectly showed it under `DYNSolverCommon.cpp`; no `calculateICCommon()` function exists in that file.
2. **Phase Reference Table (§3)**: Updated `Source` column for `PHASE_CALCULATE_IC`, `PHASE_JACOBIAN_EVAL`, `PHASE_RESIDUAL_EVAL`, and `PHASE_ROOT_EVAL` to reference `DYNSolverIDA.cpp` accurately.
3. **§4.2 Solver Layer**: Replaced the incorrect claim about `calculateICCommon()` with accurate description of `SolverIDA::calculateIC()` as the call site, with a warning about the missing fixed-step solver instrumentation.
4. **§8 Audit**: Added two new tracked items — `W-CALC_IC-IDA-ONLY` (subsequently closed) and `W-SHARED-PHASE-DOUBLE-COUNT`.

### Commit 10 — *(this commit)* — *docs(profiling): close W-CALC_IC-IDA-ONLY (false positive), upgrade W-SHARED-PHASE-DOUBLE-COUNT to cover fixed-step runs*

Second audit pass 2026-04-04 against `DYNSolverCommonFixedTimeStep.cpp`.

1. **W-CALC_IC-IDA-ONLY → CLOSED (false positive)**: `DYN_PROFILE_PHASE_MEM(PHASE_CALCULATE_IC)` is present at the top of `calculateICCommon()` in `DYNSolverCommonFixedTimeStep.cpp`. Both SolverSIM and SolverTRAP inherit this method, so `PHASE_CALCULATE_IC` is correctly instrumented for all solver types. The §1 diagram and §3/§4.2 docs now reflect all call sites accurately.

2. **W-SHARED-PHASE-DOUBLE-COUNT → broadened scope**: `PHASE_ROOT_EVAL` fires at three call sites inside `DYNSolverCommonFixedTimeStep.cpp` — inside `calculateICCommon()`, `updateZAndMode()`, and `reinit()` (×2). Because `ROOT_EVAL` accumulates globally across all call sites, the `excl(SOLVER_STEP, {..., ROOT_EVAL})` subtraction removes time that was actually incurred inside `CALCULATE_IC` and `REINIT` — not inside `SOLVER_STEP`. This deflates `SolverStep` exclusive time in **event-heavy DynaWaltz runs**, not only DynaSwing (IDA+KINSOL) runs as previously stated. Simulation correctness is unaffected.

---

## 8. Audit — Known Issues and Limitations

### Open Issues (Prioritized)

A full external review of the profiler and analysis tooling lives in the
TRAISIM repository at `reports/dynawo_code_review_2026-07-09.md`; the open
items below are complemented there by (among others):
**B-EXCL-CHILDSETS** — the report's "exclusive times" subtract hand-written
child sets that no longer match the runtime call graph, overstating
SolverStep/KINSOLSolve self-cost (fix: derive exclusives from the
`parentChildTime_` matrix); **B-EXPORT-AT-EXIT** — CSV/JSON export happens
only in the singleton destructor, so crashes lose the profile; and
**B-MAXCALLS-REWRITE** — the `MAX_PLAUSIBLE_CALLS` workaround silently
fabricates `call_count`/`avg_ms` with no warning marker.

1. **B-STEPTIME-DOUBLE (medium)**
: `stepEndTime` is captured twice per timestep when both real-time tracking and profiler timestep recording are enabled, slightly inflating both metrics.
: Location: `Simulation::simulate()` in `DYNSimulation.cpp`.
: Recommendation: capture one `stepEndTime` and reuse it for both output paths.

2. **B-THREAD-1 (medium, future-facing)**
: `SolverProfiler` is not thread-safe; this is acceptable today for single-threaded solver execution but will become risky if parallel solver/model evaluation is introduced.
: Location: profiler singleton state and all `record*()` paths.
: Recommendation: introduce per-thread accumulators (preferred) or atomic updates when multi-threading is introduced.

3. **B-MEMOTRACK-1 (low)**
: `PhaseStats::peakMemoryKB` is mainly populated for `PHASE_SIMULATION_LOOP`; fine-grained per-phase peak memory is incomplete unless explicit memory-tracked timing is added.
: Location: `DYN_PROFILE_PHASE_MEM` usage patterns and `recordWithMemory*()` call coverage.
: Recommendation: keep current behavior for low overhead, optionally add targeted memory tracking for a small set of hotspot phases.

### Recently Resolved (kept for traceability)

- `B-KLU-DEDUP` (2026-07-09): `installKLUProfiler` dedup'd on `SUNLinearSolver` pointer identity, so a solver freed and recreated at the same heap address (routine when `setupNewAlgebraicRestoration` resizes) matched its stale registry entry and was left unpatched — all subsequent klu_factor/refactor time silently vanished from `PHASE_KLU_SETUP`. Fixed: dedup on `ops->setup == profiledKLUSetup` and reuse stale slots in place (also stops registry growth on recreation). Verified with a standalone free/recreate harness (red → green).
- `B-CALC_IC-SELF-NEST` (2026-07-09): `PHASE_CALCULATE_IC` was opened both in `SolverSIM/TRAP::calculateIC()` and again in `calculateICCommon()`, self-nesting and double-counting the phase (call_count 2, up to ~2x time). Fixed by removing the inner timer; verified on a profiled Nordic run (call_count 2 → 1). Profiles recorded before this date carry the inflated values.
- `W-SHARED-PHASE-DOUBLE-COUNT`: fixed via runtime parent-child attribution in profiler timing core.
- `B-EXCL-REINIT-PARENT-MISMATCH`: fixed by contextual subtraction (not global child subtraction).
- `B-EXCL-CALC_IC`: fixed by correct SimulationLoop child mapping.
- `W-CALC_IC-IDA-ONLY`: closed as false positive after fixed-step verification.
- `B-CSV-PARSE`: fixed with explicit `# PHASES` / `# TIMESTEPS` section markers.

## 9. Usage Cookbook

### Basic Profiling Run

```bash
# 1. Build with profiling
cmake -DDYNAWO_PROFILING=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# 2. Run with CSV export
export DYNAWO_PROFILE_OUTPUT=/tmp/dynawo_profile.csv
dynawo.sh jobs mySimulation.jobs

# 3. Visualise
python3 performance-analysis/analyze_profile.py /tmp/dynawo_profile.csv
```

### JSON Export for Programmatic Analysis

None of the bundled analysis tools consume the JSON export — they parse CSV
only. Prefer CSV unless you are writing your own ad-hoc script, as below:

```bash
export DYNAWO_PROFILE_OUTPUT=/tmp/dynawo_profile.json
dynawo.sh jobs mySimulation.jobs
python3 -c "
import json
with open('/tmp/dynawo_profile.json') as f:
    d = json.load(f)
for p in sorted(d['phases'], key=lambda x: -x['total_seconds']):
    print(f\"{p['name']:20s}  {p['total_seconds']:8.3f}s  ({p['call_count']} calls)\")
"
```

### Identify KLU Hotspot with `perf`

```bash
# Build with frame pointers for better call-graph reconstruction
./myEnvDynawo.sh clean-build-dynawo \
  --cmake-options "-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer"

# Record with call-graph
mkdir -p results/nordic/perf
perf record \
  -F 1999 \
  --call-graph fp \
  -o results/nordic/perf/perf.data \
  -- ./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs

# Report: dynawo binary only call graph
perf report \
  -i results/nordic/perf/perf.data \
  --stdio \
  --dsos dynawo-1.8.0 \
  --call-graph graph,0.5 \
  > results/nordic/perf/graph_fp.txt

# Optional KLU hotspot extraction
perf report \
  -i results/nordic/perf/perf.data \
  --stdio \
  --sort comm,dso,symbol \
  --no-children \
  | grep -E 'klu_l_analyze|klu_l_factor|klu_l_refactor|klu_l_solve|btf_l_maxtrans|btf_l_strongcomp' \
  | tee results/nordic/perf/klu_hotspots.txt

# Annotate specific function
perf annotate klu_refactor
```

### Compare Two Configurations

```bash
export DYNAWO_PROFILE_OUTPUT=profile_baseline.csv
dynawo.sh jobs baseline.jobs

export DYNAWO_PROFILE_OUTPUT=profile_patched.csv
dynawo.sh jobs patched.jobs

python3 performance-analysis/compare_runs.py profile_baseline.csv profile_patched.csv
```

### Enable Real-Time CSV Alongside Profiler

In the JOB XML:
```xml
<simulation startTime="0" stopTime="60"
            enableRealTimeTracking="true"/>
```

Then also set `DYNAWO_PROFILE_OUTPUT`. Both outputs will be generated independently. The `simRT.csv` will contain wall-clock step timing with accumulated time; the profiler CSV will contain phase breakdown and per-step data with memory.

---

*Last updated: 2026-07-09 — resolved B-CALC_IC-SELF-NEST (CalculateIC double-count on SIM/TRAP) and B-KLU-DEDUP (KLUSetup timing lost after solver recreation at a reused heap address); fixed the Python tools' percentage denominators (SimulationLoop wall time, not sum of nested phases) and benchmark_solvers.py jobs-file rewiring; added the regression test suite under `performance-analysis/tests/`. See `reports/dynawo_code_review_2026-07-09.md` in the TRAISIM repository for the full review and remaining open findings.*
