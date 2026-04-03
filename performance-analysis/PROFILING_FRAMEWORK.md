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
├──────────────────────────────────────────────────────────────────────┤
│                        Solver layer                                  │
│  DYNSolverCommon.cpp                                                 │
│  ├─ DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE)                            │
│  ├─ DYN_PROFILE_PHASE(PHASE_CALCULATE_IC)                            │
│  ├─ DYN_PROFILE_PHASE(PHASE_SOLVER_STEP)                             │
│  ├─ DYN_PROFILE_PHASE(PHASE_JACOBIAN_EVAL)                           │
│  ├─ DYN_PROFILE_PHASE(PHASE_RESIDUAL_EVAL)                           │
│  ├─ DYN_PROFILE_PHASE(PHASE_ROOT_EVAL)                               │
│  ├─ DYN_PROFILE_PHASE(PHASE_MODE_EVAL)                               │
│  ├─ DYN_PROFILE_PHASE(PHASE_DISCRETE_EVAL)                           │
│  ├─ DYN_PROFILE_PHASE(PHASE_NR_SOLVE)                                │
│  ├─ DYN_PROFILE_PHASE(PHASE_MATRIX_COPY)                             │
│  ├─ DYN_PROFILE_PHASE(PHASE_KINSOL_SOLVE)                            │
│  ├─ DYN_PROFILE_PHASE(PHASE_REINIT)                                  │
│  └─ DYN_PROFILE_PHASE(PHASE_IO)         [Simulation::terminate()]    │
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
cmake -DDYNAWO_PROFILING=ON ...
```

This defines the `DYNAWO_PROFILING` preprocessor symbol, which activates all `DYN_PROFILE_*` macros and sets `SolverProfiler::enabled_ = true` in the constructor.

**Standard (non-profiling) build:**
```bash
cmake -DDYNAWO_PROFILING=OFF ...
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
├── SOLVER_SOLVE       (per solver_->solve() call)
│   ├── SOLVER_STEP        (per IDA/Euler integration step)
│   │   ├── RESIDUAL_EVAL      (F(x,x',y,t) evaluation)
│   │   ├── JACOBIAN_EVAL      (∂F/∂x + ∂F/∂ẋ assembly)
│   │   │   ├── MATRIX_COPY        (sparse matrix copy before factorization)
│   │   │   ├── KLU_SYMBOLIC       (klu_analyze — structure-changing steps)
│   │   │   └── KLU_SETUP          (SUNLinSolSetup / klu_refactor)
│   │   └── ROOT_EVAL          (zero-crossing detection)
│   ├── NR_SOLVE           (algebraic Newton-Raphson, fixed-step solver)
│   │   └── KINSOL_SOLVE       (KINSOL inner solve)
│   │       ├── RESIDUAL_EVAL  (shared with above — NOT double-counted in exclusive)
│   │       └── JACOBIAN_EVAL  (shared)
│   ├── DISCRETE_EVAL      (z-variable / event-trigger evaluation)
│   ├── MODE_EVAL          (mode detection and state machine update)
│   └── REINIT             (solver reinit after mode change)
├── CURVES_UPDATE      (updateCurves() — calculated-var flush)
├── CALCULATE_IC       (initial condition solve, outside main loop)
└── IO                 (terminate() — file export, final state dump)
```

> **Note on shared children**: `RESIDUAL_EVAL` and `JACOBIAN_EVAL` appear under both `SOLVER_STEP` and `KINSOL_SOLVE`. The exclusive-time calculation in `printReport()` subtracts them only from their **immediate parent** as declared in the `excl()` lambda, which is the correct attribution — see §8 (Bug B-EXCL-1) for a known limitation.

### Phase Reference Table

| Enum | String | Source | Triggered by |
|---|---|---|---|
| `PHASE_SIMULATION_LOOP` | `SimulationLoop` | `DYNSimulation.cpp` | Outer `simulate()` loop — memory-tracked |
| `PHASE_SOLVER_SOLVE` | `SolverSolve` | `DYNSolverCommon.cpp` | `solveStepCommon()` entry |
| `PHASE_CALCULATE_IC` | `CalculateIC` | `DYNSolverCommon.cpp` | `calculateICCommon()` entry |
| `PHASE_SOLVER_STEP` | `SolverStep` | `DYNSolverCommon.cpp` | Per IDA/Euler step |
| `PHASE_JACOBIAN_EVAL` | `JacobianEval` | `DYNSolverCommon.cpp` | Jacobian callback |
| `PHASE_RESIDUAL_EVAL` | `ResidualEval` | `DYNSolverCommon.cpp` | Residual callback |
| `PHASE_ROOT_EVAL` | `RootEval` | `DYNSolverCommon.cpp` | Zero-crossing callback |
| `PHASE_MODE_EVAL` | `ModeEval` | `DYNSolverCommon.cpp` | Mode detection |
| `PHASE_DISCRETE_EVAL` | `DiscreteEval` | `DYNSolverCommon.cpp` | Discrete variable update |
| `PHASE_NR_SOLVE` | `NRSolve` | `DYNSolverCommon.cpp` | Fixed-step algebraic solve |
| `PHASE_MATRIX_COPY` | `MatrixCopy` | `DYNSolverCommon.cpp` | Pre-factorization matrix copy |
| `PHASE_KINSOL_SOLVE` | `KINSOLSolve` | `DYNSolverCommon.cpp` | KINSOL inner solve |
| `PHASE_REINIT` | `Reinit` | `DYNSolverCommon.cpp` | Post-mode-change solver reinit |
| `PHASE_IO` | `IO` | `DYNSimulation.cpp` | `Simulation::terminate()` |
| `PHASE_KLU_SYMBOLIC` | `KLUSymbolic` | KLU ops-patch | `klu_analyze` (new sparsity structure) |
| `PHASE_KLU_SETUP` | `KLUSetup` | KLU ops-patch | `SUNLinSolSetup` → `klu_refactor` |
| `PHASE_CURVES_UPDATE` | `CurvesUpdate` | `DYNSimulation.cpp` | `updateCurves()` after early-return guard |

---

## 4. Instrumentation Points

### 4.1 Simulation Layer (`DYNSimulation.cpp`)

**`Simulation::simulate()`**

```cpp
// Outer loop — memory-tracked
DYN_PROFILE_PHASE_MEM(PHASE_SIMULATION_LOOP);   // RAII, fires at '}' of inner scope

// Per-timestep timing — independent of real-time tracking CSV
auto stepStartTime = std::chrono::high_resolution_clock::now(); // declared unconditionally
// ... solver_->solve() ...

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

**`Simulation::updateCurves()`**

```cpp
void Simulation::updateCurves(const bool updateCalculatedVariable) const {
  if (exportCurvesMode_ == EXPORT_CURVES_NONE &&
      exportFinalStateValuesMode_ == EXPORT_FINAL_STATE_VALUES_NONE)
    return;               // ← zero cost if no curves configured
  DYN_PROFILE_PHASE(PHASE_CURVES_UPDATE);  // ← B3b: after guard
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

### 4.2 Solver Layer (`DYNSolverCommon.cpp`)

The common solver base class instruments all callbacks that SUNDIALS IDA and the fixed-step Backward Euler solver invoke:

```cpp
void SolverCommon::solveStepCommon(...) {
  DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE);
  // ...delegates to IDA or Euler step...
}

void SolverCommon::calculateICCommon(...) {
  DYN_PROFILE_PHASE(PHASE_CALCULATE_IC);
  // ...
}
```

Inner callbacks (residual, Jacobian, root, discrete, mode) are wrapped with `DYN_PROFILE_PHASE(...)` at their respective entry points in the SUNDIALS callback functions registered via `IDASetUserData` / `IDASetJacFn` etc.

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

**Exclusive breakdown** — shows `Phase | Excl(s) | Pct(%)` for the six phases that have instrumented children.

**Timestep count** and **Peak RSS** are appended at the bottom.

### 5.2 CSV Export (`DYNAWO_PROFILE_OUTPUT=<file>.csv`)

Two sections in a single file:

**Section 1 — Phase summary:**
```
phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb
SimulationLoop,12.345678,1,12345.6780,12345.6780,12345.6780,204800
SolverSolve,11.234567,1000,11.2346,...
...
```

**Section 2 — Timestep time-series (appended after a blank line):**
```
sim_time,step_duration_ms,memory_kb
0.000000,1.2340,204800
0.010000,1.1890,204816
...
```

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

**Important (post B4)**: `DYN_PROFILE_RECORD_TIMESTEP` is now executed in its own `#ifdef DYNAWO_PROFILING` scope, independently of `enableRealTimeTracking_`. Both features share the same `stepStartTime` variable (declared unconditionally before `solver_->solve()`). The `stepEndTime` variable is computed twice (once per feature when both are active) — this is intentional to keep both features decoupled and avoid ordering dependency.

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

### Commit 6 — `f1b880` — *fix(profiler): instrument solveStepCommon (B1) and calculateICCommon (B2) with profiler phases*

Added `DYN_PROFILE_PHASE(PHASE_SOLVER_SOLVE)` at the entry of `SolverCommon::solveStepCommon()` and `DYN_PROFILE_PHASE(PHASE_CALCULATE_IC)` at the entry of `SolverCommon::calculateICCommon()`. These are the two most important coarse-grained instrumentation points for separating initialization cost from simulation loop cost.

### Commit 7 — `4f7340a` — *profiler: fix B3b + B4 — instrument updateCurves() and decouple RECORD_TIMESTEP from enableRealTimeTracking_*

1. **B3b**: Added `DYN_PROFILE_PHASE(PHASE_CURVES_UPDATE)` inside `Simulation::updateCurves()` after the early-return guard. Closes the gap between the enum definition (B3a) and actual instrumentation.
2. **B4**: Moved `DYN_PROFILE_RECORD_TIMESTEP` out of the `if (enableRealTimeTracking_)` block into a standalone `#ifdef DYNAWO_PROFILING` scope, so profiling builds always record per-timestep entries regardless of JOB XML settings.

---

## 8. Audit — Known Issues and Limitations

### 🔴 BUG — B-CSV-PARSE: Two-section CSV breaks standard parsers

**Location**: `SolverProfiler::exportCSV()` in `DYNSolverProfiler.cpp`

**Description**: The CSV file contains two independent tables separated by a blank line, with different column counts (7 vs 3). Any standard CSV reader (`pandas.read_csv`, `csv.reader`) will fail on the second table or interpret blank lines as errors.

**Consequence**: `analyze_profile.py` must implement custom parsing to skip the blank line and detect section boundaries. If a user opens the file with Excel or a naive parser they will get garbled output.

**Recommended fix**: Output two separate files — `profile_phases.csv` and `profile_timesteps.csv` — or add a dedicated section-header comment line:
```
# PHASES
phase,total_seconds,...
...
# TIMESTEPS
sim_time,step_duration_ms,memory_kb
...
```

---

### 🔴 BUG — B-MINTIME-INIT: `minTime` initialised to `+∞` but never guarded in report

**Location**: `PhaseStats::reset()` initialises `minTime = std::numeric_limits<double>::max()`. The `printReport()` method guards it:
```cpp
double minMs = (s.minTime < std::numeric_limits<double>::max()) ? s.minTime * 1000.0 : 0.0;
```
But `exportCSV()` and `exportJSON()` apply the same guard. **This is actually correct** — however `exportJSON()` applies the guard only for `min_ms`:
```cpp
double minMs = (s.minTime < ...) ? s.minTime * 1000.0 : 0.0;
```
If a phase is recorded exactly **once**, `minTime == maxTime == elapsedSeconds` and the guard fires correctly. **However**, if the phase is entered but the `PhaseTimer` is destroyed before `record()` is called (e.g., early-return inside the timed scope via exception), `callCount > 0` but `minTime` remains `+∞` in the CSV/JSON output.

**Recommended fix**: In `SolverProfiler::record()`, always clamp `minTime`:
```cpp
void SolverProfiler::record(ProfilePhase phase, double elapsedSeconds) {
  if (!enabled_) return;
  PhaseStats& s = stats_[phase];
  s.totalTime += elapsedSeconds;
  s.callCount++;
  if (elapsedSeconds < s.minTime) s.minTime = elapsedSeconds;  // always fires on first call
  if (elapsedSeconds > s.maxTime) s.maxTime = elapsedSeconds;
}
```
This is already correct — the guard is defensive only for the zero-call case. The real issue is that `callCount` can become non-zero without `minTime` being set if `PhaseTimer::~PhaseTimer()` calls `record()` with `elapsedSeconds = 0.0` due to a zero-duration race. This is highly unlikely in practice but worth noting.

---

### 🟡 LIMITATION — B-EXCL-1: Exclusive-time under-counts when KINSOL is active

**Location**: `SolverProfiler::printReport()` exclusive-time lambda

**Description**: `RESIDUAL_EVAL` and `JACOBIAN_EVAL` are subtracted from both `SOLVER_STEP` and `KINSOL_SOLVE` via separate `excl()` calls. However, these two phases may be entered from both contexts in the same simulation run. The exclusive calculation assumes a strict single-parent model and does not apportion time proportionally.

**Consequence**: In DynaSwing simulations using KINSOL for algebraic initialisation, `SolverStep` exclusive time may appear slightly negative or misleadingly small.

**Recommendation**: Add a note in the report output header: *"Exclusive times assume single-parent nesting; shared phases (ResidualEval, JacobianEval) may be double-counted under mixed IDA+KINSOL runs."*

---

### 🟡 LIMITATION — B-THREAD-1: Singleton not thread-safe

**Location**: `SolverProfiler::instance()` and all `record*()` methods

**Description**: `SolverProfiler` is documented as *"not thread-safe; designed for single-threaded solver execution."* This is intentional given Dynaωo's current single-threaded solver architecture. However, future parallelisation work (OpenMP Jacobian assembly, parallel model evaluation) could trigger data races on `stats_[phase]` and `timestepRecords_`.

**Recommendation**: When OpenMP instrumentation is added in future, wrap `record()` with `#pragma omp atomic` for the `totalTime` and `callCount` fields, or use `std::atomic<double>` / per-thread accumulators with reduction.

---

### 🟡 LIMITATION — B-STEPTIME-DOUBLE: `stepEndTime` computed twice when both features active

**Location**: `Simulation::simulate()`, after `solver_->solve()`

**Description**: When both `enableRealTimeTracking_ = true` AND `DYNAWO_PROFILING` is defined, two separate `chrono::high_resolution_clock::now()` calls are made per timestep — one inside `if (enableRealTimeTracking_)` and one inside `#ifdef DYNAWO_PROFILING`. The two timestamps are microseconds apart and will slightly overstate both durations.

**Recommended fix**: Capture a single `stepEndTime` unconditionally (mirroring `stepStartTime`):
```cpp
// After solver_->solve() and all step work:
auto stepEndTime = std::chrono::high_resolution_clock::now();
if (enableRealTimeTracking_) {
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stepEndTime - stepStartTime);
  double stepTimeMs = dur.count() / 1000.0;
  auto accDur = std::chrono::duration_cast<std::chrono::microseconds>(stepEndTime - simulationStartTime_);
  double accTimeS = accDur.count() / 1000000.0;
  timingData_.emplace_back(tCurrent_, stepTimeMs, accTimeS);
}
#ifdef DYNAWO_PROFILING
{
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stepEndTime - stepStartTime);
  DYN_PROFILE_RECORD_TIMESTEP(tCurrent_, dur.count() / 1000.0,
                              DYN::SolverProfiler::getCurrentMemoryKB());
}
#endif
```

---

### 🟡 LIMITATION — B-MEMOTRACK-1: Memory only tracked on `PHASE_SIMULATION_LOOP`

**Description**: `DYN_PROFILE_PHASE_MEM(PHASE_SIMULATION_LOOP)` uses `trackMemory = true`, which calls `getCurrentMemoryKB()` once at the end of the loop. Only `peakMemoryKB` for `PHASE_SIMULATION_LOOP` is populated via this path. Sub-phases (`PHASE_KLU_SETUP`, etc.) never populate `peakMemoryKB` unless `recordWithMemory()` is called explicitly.

**Consequence**: The *"Peak RSS"* reported at the bottom of `printReport()` scans all phases but will only find non-zero in `PHASE_SIMULATION_LOOP`, making the scan redundant.

**Recommendation**: Either instrument critical sub-phases with `DYN_PROFILE_PHASE_MEM(...)` (note: `fopen("/proc/self/status")` per step is expensive), or capture memory only in `DYN_PROFILE_RECORD_TIMESTEP` (already done) and report peak from the timestep series rather than from `PhaseStats::peakMemoryKB`.

---

### 🟢 NOTE — B-PROCFS-COST: `/proc/self/status` read in `getCurrentMemoryKB()`

**Description**: `DYN_PROFILE_RECORD_TIMESTEP` calls `SolverProfiler::getCurrentMemoryKB()` per timestep, which opens and scans `/proc/self/status`. On Linux this is a virtual file backed by the kernel's task structure — typically ~2–5 µs per call. For a 10-second simulation with 1 ms average timestep (~10,000 steps), this adds ~20–50 ms to total runtime (~0.2–0.5% overhead). Acceptable for profiling builds.

**Recommendation**: None required. Acceptable overhead for a profiling build. Document in user-facing README.

---

### 🟢 NOTE — B-REINIT-SCOPE: `PHASE_REINIT` declared but instrumentation not confirmed

**Description**: `PHASE_REINIT` appears in the enum and `phaseToString()` but its placement in `DYNSolverCommon.cpp` was not explicitly part of the 7-commit series. If `solver_->reinit()` in `simulate()` is not wrapped, `callCount` will be zero and it will not appear in the report — which is silently correct behaviour but can create confusion during analysis.

**Recommendation**: Verify that `DYN_PROFILE_PHASE(PHASE_REINIT)` is placed at the entry of the solver's `reinit()` implementation in `DYNSolverCommon.cpp`.

---

### 🟢 NOTE — B-KINSOL-PLACEMENT: `PHASE_KINSOL_SOLVE` and `PHASE_NR_SOLVE` exclusive correctness

**Description**: The exclusive-time breakdown subtracts `KINSOL_SOLVE` from `NR_SOLVE`. This is correct for DynaWaltz (long-term stability) where KINSOL is used inside the NR fixed-step algebraic solve. For DynaSwing (IDA-based), `PHASE_NR_SOLVE` should have zero calls and the subtraction is a no-op. Confirmed safe.

---

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

```bash
export DYNAWO_PROFILE_OUTPUT=/tmp/dynawo_profile.json
dynawo.sh jobs mySimulation.jobs
python3 -c "
import json
with open('/tmp/dynawo_profile.json') as f:
    d = json.load(f)
for p in sorted(d['phases'], key=lambda x: -x['total_seconds']):
    print(f"{p['name']:20s}  {p['total_seconds']:8.3f}s  ({p['call_count']} calls)")
"
```

### Identify KLU Hotspot with `perf`

```bash
# Build with frame pointers for better call-graph reconstruction
cmake -DDYNAWO_PROFILING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer" ..
make -j$(nproc)

# Record with call-graph
perf record -g --call-graph dwarf -- dynawo.sh jobs mySimulation.jobs

# Report: focus on KLU
perf report --no-children -g --sort symbol | grep -A5 klu

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

*Last updated: April 2026 — reflects commits `1dce6fd`…`4f7340a` on `3_performance-analysis-framework`.*
