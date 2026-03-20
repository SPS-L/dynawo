# Dynawo Performance Analysis Framework

A comprehensive profiling and benchmarking framework for the Dynawo power system simulation tool. This framework instruments the solver pipeline, collects per-phase timing and memory data, and provides Python-based analysis tools for visualization and regression detection.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Architecture Overview](#architecture-overview)
3. [Components](#components)
4. [Directory Structure](#directory-structure)
5. [Enabling Profiling](#enabling-profiling)
6. [Running Benchmarks](#running-benchmarks)
7. [Interpreting Results](#interpreting-results)
8. [Adding New Instrumentation Points](#adding-new-instrumentation-points)
9. [Example Workflow](#example-workflow)

---

## Quick Start

For full build-from-source instructions on Ubuntu 24.04 (dependencies, clone,
build, profiling build, troubleshooting), see
**[INSTALL_UBUNTU24.md](INSTALL_UBUNTU24.md)**.

For the optimization roadmap and future plans, see
**[OPTIMIZATION_ROADMAP.md](OPTIMIZATION_ROADMAP.md)** and
**[TRAISIM_future_plans.md](TRAISIM_future_plans.md)** (Marp presentation).

The rest of this README documents the profiling framework architecture,
Python analysis tools, benchmark infrastructure, and data formats.

---

## Architecture Overview

The performance analysis framework operates at three levels:

```
+------------------------------------------------------------------+
|                    Dynawo Simulation Runtime                      |
|                                                                   |
|  +------------------+    +-------------------+    +-------------+ |
|  | SolverProfiler   |    | PhaseTimer (RAII) |    | Profiling   | |
|  | (Singleton)      |<---| Scoped timers     |    | Macros      | |
|  | Collects stats   |    | in solver code    |    | Zero-cost   | |
|  +--------+---------+    +-------------------+    | when OFF    | |
|           |                                       +-------------+ |
+-----------|-------------------------------------------------------+
            |  CSV / JSON export
            v
+------------------------------------------------------------------+
|                    Analysis Layer (Python)                        |
|                                                                   |
|  +------------------+    +-------------------+    +-------------+ |
|  | pandas / numpy   |    | matplotlib        |    | Jinja2      | |
|  | Data loading     |    | Visualization     |    | HTML report | |
|  | Statistical      |    | Phase breakdown   |    | generation  | |
|  | analysis         |    | Time series       |    |             | |
|  +------------------+    +-------------------+    +-------------+ |
+------------------------------------------------------------------+
            |
            v
+------------------------------------------------------------------+
|                    Benchmark Layer (Shell/Python)                 |
|                                                                   |
|  +------------------+    +-------------------+    +-------------+ |
|  | Benchmark runner |    | Solver configs    |    | Regression  | |
|  | Iterates test    |    | IDA / SIM params  |    | detection   | |
|  | cases            |    |                   |    |             | |
|  +------------------+    +-------------------+    +-------------+ |
+------------------------------------------------------------------+
```

**C++ Profiler Layer:** The `SolverProfiler` singleton and `PhaseTimer` RAII class live in the Dynawo solver source code (`dynawo/sources/Solvers/Common/`). When enabled, they measure wall-clock time and RSS memory for each solver phase using `std::chrono::high_resolution_clock` and `/proc/self/status`. All data collection is guarded by preprocessor macros that compile to no-ops when profiling is disabled, ensuring zero overhead in production builds.

**Python Analysis Layer:** Python scripts load the exported CSV or JSON files, compute statistics, generate visualizations (phase pie charts, time-series plots, memory growth curves), and produce HTML reports. Dependencies are listed in `requirements.txt`.

**Benchmark Layer:** Shell scripts and solver configuration files automate running standardized test cases with consistent solver settings, collecting profiling data across runs for comparison. The primary benchmark is the Nordic test system (74 buses, 52 lines, 20 generators, 22 loads, 41 dynamic models, 175s DynaWaltz simulation with fault event), with NordicTCB as a secondary case.

---

## Components

### C++ Profiler (`DYNSolverProfiler.h` / `.cpp`)

Located at `dynawo/sources/Solvers/Common/`:

- **`ProfilePhase` enum:** Defines the measurable phases of the solver pipeline:
  - `PHASE_SIMULATION_LOOP` -- outermost simulation loop
  - `PHASE_SOLVER_SOLVE` -- top-level solve call
  - `PHASE_CALCULATE_IC` -- initial condition calculation
  - `PHASE_SOLVER_STEP` -- individual time step
  - `PHASE_JACOBIAN_EVAL` -- Jacobian matrix evaluation
  - `PHASE_RESIDUAL_EVAL` -- residual function (F) evaluation
  - `PHASE_ROOT_EVAL` -- root/zero-crossing evaluation
  - `PHASE_MODE_EVAL` -- mode change evaluation
  - `PHASE_DISCRETE_EVAL` -- discrete variable evaluation
  - `PHASE_NR_SOLVE` -- Newton-Raphson algebraic solve (SIM/TRAP fixed-timestep solvers)
  - `PHASE_MATRIX_COPY` -- matrix data copy operations
  - `PHASE_KINSOL_SOLVE` -- KINSOL nonlinear solve
  - `PHASE_REINIT` -- solver reinitialization after events
  - `PHASE_IO` -- file I/O operations (output writing in terminate())

- **`PhaseStats` struct:** Stores per-phase statistics (total time, min/max/avg call time, call count, peak memory).

- **`SolverProfiler` class (singleton):** Central data collector. Provides `record()`, `recordWithMemory()`, and `recordTimestep()` methods. Exports to CSV or JSON via `exportCSV()` and `exportJSON()`. Automatically exports on destruction if `DYNAWO_PROFILE_OUTPUT` environment variable is set.

- **`PhaseTimer` class (RAII):** Construct at the top of a scope to automatically measure its duration. The destructor records the elapsed time to the global profiler. Optionally tracks memory via `/proc/self/status`.

- **Profiling macros:**
  - `DYN_PROFILE_PHASE(phase)` -- create a scoped timer for a phase
  - `DYN_PROFILE_PHASE_MEM(phase)` -- scoped timer with memory tracking
  - `DYN_PROFILE_RECORD_TIMESTEP(simTime, stepMs, memKB)` -- record a timestep entry
  - `DYN_PROFILE_PRINT_REPORT()` -- print summary to the Dynawo trace log
  - `DYN_PROFILE_EXPORT_CSV(filename)` -- export CSV
  - `DYN_PROFILE_EXPORT_JSON(filename)` -- export JSON

### Python Analysis Tools

Located in `performance-analysis/`. Install dependencies first:

```bash
cd /path/to/dynawo/performance-analysis
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt   # matplotlib, pandas, numpy, jinja2
```

#### `analyze_profile.py` — Profile Visualization

Parses a profiler CSV export and generates charts plus a console summary table.

**Usage:**

```bash
python analyze_profile.py <profile.csv> [--output-dir <dir>]
```

| Argument | Description |
|----------|-------------|
| `profile.csv` | Path to the profiler CSV export (positional, required) |
| `--output-dir` | Directory for output charts (default: current directory) |

**Output files:**

| File | Description |
|------|-------------|
| `time_breakdown_pie.png` | Pie chart showing time distribution across solver phases |
| `hotspot_bar.png` | Bar chart ranking phases by total time (top hotspots) |
| `step_duration_ts.png` | Time-series plot of per-timestep solver duration |
| `memory_usage_ts.png` | Time-series plot of memory (RSS) over simulation time |

Also prints a summary table to stdout with per-phase statistics.

**Example:**

```bash
python analyze_profile.py results/nordic/profile.csv --output-dir results/nordic/charts
```

#### `compare_runs.py` — Run Comparison & HTML Report

Compares two profiling runs (baseline vs. optimised), computes per-phase speedup
ratios, flags regressions, and generates a self-contained HTML report.

**Usage:**

```bash
python compare_runs.py <baseline.csv> <optimized.csv> [--output <report.html>]
```

| Argument | Description |
|----------|-------------|
| `baseline.csv` | Path to the baseline profiler CSV (positional, required) |
| `optimized.csv` | Path to the optimised profiler CSV (positional, required) |
| `--output` | Output HTML report path (default: `comparison_report.html`) |

**Output:**
- Console table with per-phase timing comparison and speedup
- HTML report with colour-coded regression/improvement table, timestep summaries,
  and a regressions section

**Example:**

```bash
# Compare baseline vs. optimised run
python compare_runs.py results/nordic/profile_baseline.csv \
                       results/nordic/profile_optimized.csv \
                       --output results/nordic/comparison.html
```

#### `benchmark_solvers.py` — Multi-Configuration Benchmarking

Automates running the same simulation case under multiple solver configurations
(IDA, SIM, TRAP with various tolerances), collects profiling data, and produces
a comparison table. Optionally runs parameter sensitivity sweeps.

**Usage:**

```bash
python benchmark_solvers.py --dynawo-bin <path> --case <case_dir> \
    [--output-dir <dir>] [--configs <name> ...] [--sensitivity]
```

| Argument | Description |
|----------|-------------|
| `--dynawo-bin` | Path to the Dynawo binary or launcher script (required) |
| `--case` | Path to the Dynawo case directory containing a `.jobs` file (required) |
| `--output-dir` | Directory for benchmark outputs (default: `benchmark_output`) |
| `--configs` | Names of configurations to run (default: all). Available: `IDA_default`, `IDA_tight_tol`, `IDA_loose_tol`, `SIM_default`, `SIM_small_step`, `TRAP_default`, `IDA_high_order` |
| `--sensitivity` | Also run parameter sensitivity sweeps (absAccuracy, maxStep, hMax) |

**Output:**
- Per-configuration profile CSV files in `<output-dir>/<config_name>/`
- Console comparison table (phase timings across all configurations)
- `benchmark_results.json` with raw results for further processing

**Example:**

```bash
# Run all solver configurations on the Nordic case
python benchmark_solvers.py \
    --dynawo-bin /path/to/dynawo.sh \
    --case examples/DynaWaltz/Nordic \
    --output-dir results/benchmark

# Run only IDA configs with sensitivity analysis
python benchmark_solvers.py \
    --dynawo-bin /path/to/dynawo.sh \
    --case examples/DynaWaltz/Nordic \
    --configs IDA_default IDA_tight_tol IDA_loose_tol \
    --sensitivity
```

#### `bottleneck_detector.py` — Automatic Bottleneck Identification

Applies heuristic rules to a profiling CSV to automatically identify performance
bottlenecks and produce prioritised recommendations with severity levels.

**Checks performed:**
1. Dominant hotspots — phases consuming >10% of total time
2. Excessive Jacobian rebuilds relative to solver steps
3. Linear solver efficiency — factorisation-to-solve ratio
4. Convergence issues — too many KINSOL solves per solver step
5. Re-initialisation frequency
6. Timestep variability and outlier detection
7. I/O overhead

**Severity levels:** `CRITICAL` (primary performance limiter), `WARNING` (notable
issue), `INFO` (observation that may help with tuning).

**Usage:**

```bash
python bottleneck_detector.py <profile.csv>
```

| Argument | Description |
|----------|-------------|
| `profile.csv` | Path to the profiler CSV export (positional, required) |

**Example:**

```bash
python bottleneck_detector.py results/nordic/profile.csv
```

Sample output:

```
==============================================================================
BOTTLENECK ANALYSIS REPORT
==============================================================================

--- CRITICAL ---

  [1] Hotspot (JacobianEval)
      Phase 'JacobianEval' consumes 42.3% of total time (4.5670s / 10.8000s).
      Recommendation: Consider increasing the Jacobian reuse count ...

--- WARNING ---

  [2] Jacobian Rebuilds (JacobianEval)
      Jacobian evaluated 850 times for 200 solver steps (ratio 4.25). ...

==============================================================================
Total findings: 3 (1 CRITICAL, 1 WARNING, 1 INFO)
==============================================================================
```

#### `memory_analyzer.py` — Memory Analysis & Leak Detection

Analyses memory usage patterns: peak memory per phase, leak detection via linear
regression on the timestep memory time-series, and optional Jacobian matrix memory
footprint estimation.

**Usage:**

```bash
python memory_analyzer.py <profile.csv> [--output-dir <dir>] \
    [--problem-size <n>] [--nnz <n>]
```

| Argument | Description |
|----------|-------------|
| `profile.csv` | Path to the profiler CSV export (positional, required) |
| `--output-dir` | Directory for output charts (default: current directory) |
| `--problem-size` | Number of state variables, for Jacobian memory estimation |
| `--nnz` | Number of non-zeros in Jacobian, for sparse memory estimation |

**Output files:**

| File | Description |
|------|-------------|
| `memory_timeline.png` | Memory usage timeline with peak annotation and mean line |
| `memory_leak_detection.png` | Memory time-series with linear regression overlay |

Also prints a peak memory per-phase table and leak detection report to stdout.

**Example:**

```bash
# Basic memory analysis
python memory_analyzer.py results/nordic/profile.csv \
    --output-dir results/nordic/memory

# With Jacobian footprint estimation (e.g., 500 state variables, 5000 non-zeros)
python memory_analyzer.py results/nordic/profile.csv \
    --output-dir results/nordic/memory \
    --problem-size 500 --nnz 5000
```

### Benchmark Infrastructure

Located in `performance-analysis/benchmarks/`:

- **`solver_configs/`:** Predefined solver parameter files for consistent benchmarking across different solver configurations (IDA parameters, time step settings, tolerance levels).

---

## Directory Structure

```
performance-analysis/
|-- README.md                    # This file
|-- INSTALL_UBUNTU24.md          # Build guide for Ubuntu 24.04
|-- OPTIMIZATION_ROADMAP.md      # Detailed optimization roadmap (16 items, phased)
|-- TRAISIM_future_plans.md      # Marp slide deck — optimization roadmap presentation
|-- requirements.txt             # Python dependencies
|-- analyze_profile.py           # CSV/JSON parser, charts, statistics
|-- compare_runs.py              # Side-by-side run comparison, HTML report
|-- benchmark_solvers.py         # Automated multi-config benchmarking
|-- bottleneck_detector.py       # Automatic bottleneck identification
|-- memory_analyzer.py           # Memory growth analysis, leak detection
|-- benchmarks/
|   |-- run_benchmarks.sh        # Master benchmark orchestration script
|   |-- Nordic_IDA.jobs          # Nordic test with IDA solver
|   |-- solver_configs/          # Predefined solver configurations
|   |   |-- sim_nordic_baseline.par, sim_nordic_fast.par, ...
|   |   |-- ida_nordic.par, ida_default.par, ...

dynawo/sources/Solvers/Common/
|-- DYNSolverProfiler.h          # Profiler header (enums, classes, macros)
|-- DYNSolverProfiler.cpp        # Profiler implementation (stats, export)
```

---

## Enabling Profiling

### Method 1: Build-Time Activation (Recommended)

After an initial `./myEnvDynawo.sh build-user`, add the following lines to
your `myEnvDynawo.sh` (before the `$DYNAWO_HOME/util/envDynawo.sh` call):

```bash
# RelWithDebInfo: optimised build with debug symbols (compatible with perf, Valgrind)
export DYNAWO_RELEASE_WITH_DEBUG=true

# Enable solver profiling instrumentation at compile time
export DYNAWO_CMAKE_OPTIONAL="-DDYNAWO_PROFILING=ON"
```

Then do a clean rebuild (third-party libraries do not need to be rebuilt):

```bash
./myEnvDynawo.sh clean-build-dynawo
```

This defines the `DYNAWO_PROFILING` preprocessor macro, which activates all `DYN_PROFILE_PHASE` and related macros throughout the solver code. Without this flag, these macros expand to `((void)0)` and are optimized away completely.

For the complete build-from-source workflow (dependencies, clone, environment
setup, profiling verification, and troubleshooting), see
[INSTALL_UBUNTU24.md](INSTALL_UBUNTU24.md).

### Runtime Behaviour

When built with `-DDYNAWO_PROFILING=ON`, profiling is always active — there is no runtime switch to disable it. All `PhaseTimer` macros are compiled in and the `SolverProfiler` singleton is unconditionally enabled.

In a non-profiling build (the default), all profiling macros expand to `((void)0)` and are optimised away completely, so there is zero overhead regardless of environment variables.

### Automatic Export

Set `DYNAWO_PROFILE_OUTPUT` to automatically write profiling data when the simulation finishes:

```bash
export DYNAWO_PROFILE_OUTPUT=/path/to/results/profile.csv
# or for JSON:
export DYNAWO_PROFILE_OUTPUT=/path/to/results/profile.json
```

The file extension determines the format. The profiler writes data in its destructor at process exit.

### Timestep Time-Series Requirement

The timestep time-series section (per-step durations and memory) is only recorded when the job's `enableRealTimeTracking` option is set to `true` in the simulation entry of the `.jobs` XML file. Without this setting, only the phase summary section will be populated; the Python tools (`analyze_profile.py` step duration and memory plots, `memory_analyzer.py` leak detection) require the time-series data to produce timestep-level output.

---

## Running Benchmarks

### Basic Benchmark Run

```bash
# 1. Set up the environment
export DYNAWO_PROFILING=1
cd $DYNAWO_HOME

# 2. Create results directory
mkdir -p results/nordic

# 3. Run with profiling export
export DYNAWO_PROFILE_OUTPUT=results/nordic/profile.csv
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs

# 4. View the summary (printed to stdout during execution)
```

### Nordic Test System

The primary benchmark is the Nordic test system, a DynaWaltz long-term stability simulation:

- **Network:** 74 buses, 52 lines, 20 generators, 22 loads
- **Dynamic models:** 41 models (governors, AVRs, OELs, loads, etc.)
- **Simulation:** 0-175s with a NodeFault at bus 4032_401 and line disconnection
- **Solver:** SolverSIM (fixed-step) with algebraic restoration via KINSOL
- **Location:** `examples/DynaWaltz/Nordic/Nordic.jobs`

The NordicTCB variant (`examples/DynaWaltz/NordicTCB/NordicTCB.jobs`) adds Transformer Current Blocking with a 300s duration.

### Comparing IDA vs SIM Solvers

A Nordic_IDA.jobs variant is provided in `benchmarks/` to run the same Nordic system with the IDA (variable-step) solver instead of SIM (fixed-step). This enables direct comparison of solver strategies:

```bash
# Run with SIM solver (default)
export DYNAWO_PROFILE_OUTPUT=results/nordic/profile_sim.csv
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs

# Run with IDA solver
export DYNAWO_PROFILE_OUTPUT=results/nordic/profile_ida.csv
./myEnvDynawo.sh jobs performance-analysis/benchmarks/Nordic_IDA.jobs
```

### Using Solver Configurations

The `benchmarks/solver_configs/` directory contains predefined solver parameter files for consistent benchmarking:

- `sim_nordic_baseline.par` -- baseline SIM config (matches Nordic/Solver.par)
- `sim_nordic_fast.par` -- looser tolerances (fnormtol=1e-2, mxiter=10)
- `sim_nordic_accurate.par` -- tighter tolerances (fnormtol=1e-4, mxiter=30)
- `ida_nordic.par` -- IDA solver config for Nordic

### Multiple-Run Benchmarking

For statistical significance, run each benchmark multiple times:

```bash
for i in $(seq 1 5); do
    export DYNAWO_PROFILE_OUTPUT=results/nordic/run_${i}.csv
    ./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
done
```

---

## Interpreting Results

### Phase Breakdown

The profiler summary table shows each phase with the following columns:

| Column | Description |
|--------|-------------|
| Phase | Name of the solver phase |
| Total(s) | Total accumulated wall-clock time in seconds |
| Calls | Number of times this phase was entered |
| Avg(ms) | Average duration per call in milliseconds |
| Min(ms) | Fastest single call in milliseconds |
| Max(ms) | Slowest single call in milliseconds |
| Pct(%) | Percentage of total simulation time |

### What Each Phase Means

- **SimulationLoop:** The outermost timing scope. All other phases are contained within this. This is the total wall-clock time of the simulation.

- **SolverSolve:** The top-level call to the numerical solver (IDA or simplified solver). This encompasses all numerical work.

- **CalculateIC:** Initial condition calculation at the start of the simulation. A large value here may indicate difficulties finding a consistent initial state.

- **SolverStep:** Individual time step computations. The sum of all SolverStep times should be close to SolverSolve minus CalculateIC.

- **JacobianEval:** Evaluation of the Jacobian matrix (partial derivatives of the DAE system). This is typically one of the most expensive phases. High percentage here (>30%) suggests the Jacobian evaluation is a bottleneck.

- **ResidualEval:** Evaluation of the residual function F(y, y'). Called at every Newton iteration within each time step. High call counts are normal.

- **RootEval:** Zero-crossing detection for event handling. Frequent calls indicate many discrete events in the simulation.

- **ModeEval:** Evaluation of mode changes (topology changes, protection relay actions). Typically infrequent but can be expensive per call.

- **DiscreteEval:** Evaluation of discrete variables. Related to event handling and mode changes.

- **NRSolve:** Newton-Raphson algebraic solve. Only appears in SIM/TRAP (fixed-timestep) solver output. This covers the full KINSOL-based NR iteration including residual evaluation, Jacobian factorization, and linear solves. For IDA (variable-timestep), there is no separate linear solve phase because the KLU solve happens internally within SUNDIALS and its time is included in `SolverStep`.

- **MatrixCopy:** Copying matrix data between internal representations. Should be a small fraction; if not, this indicates unnecessary data movement.

- **KINSOLSolve:** KINSOL nonlinear solver calls (used for initial condition calculation and event reinitialization).

- **Reinit:** Solver reinitialization after mode changes or events. Frequent reinitializations indicate many discrete events.

- **IO:** File I/O operations during `terminate()` (writing curves, timeline, constraints, IIDM state, dump files). Should be small unless output is very large.

### Key Metrics to Watch

1. **JacobianEval + NRSolve (SIM/TRAP) combined percentage:** If these together exceed 60% of simulation time, the simulation is compute-bound on sparse linear algebra. For IDA, JacobianEval alone is the key indicator since the KLU linear solve time is embedded in SolverStep. Consider the optimization strategies in `OPTIMIZATION_ROADMAP.md`.

2. **JacobianEval call count vs. SolverStep call count:** If Jacobian evaluations are much more frequent than time steps, the Newton solver may be struggling to converge. Check tolerance settings.

3. **Max vs. Avg for SolverStep:** A large ratio (max >> avg) indicates occasional difficult time steps, possibly near events or discontinuities.

4. **Memory growth:** If peak RSS increases significantly over the simulation, there may be a memory leak or accumulating data structures.

5. **Reinit frequency:** Many reinitialization events can dominate simulation time in event-heavy scenarios.

### CSV Format

The CSV export contains two sections:

**Phase summary (first section):**
```csv
phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb
SimulationLoop,45.123456,1,45123.4560,45123.4560,45123.4560,524288
JacobianEval,18.567890,1250,14.8543,8.2100,45.6700,0
...
```

**Timestep time series (second section, separated by a blank line):**
```csv
sim_time,step_duration_ms,memory_kb
0.000000,12.3456,262144
0.010000,8.7654,262148
...
```

### JSON Format

```json
{
  "phases": [
    {
      "name": "SimulationLoop",
      "total_seconds": 45.123456,
      "call_count": 1,
      "avg_ms": 45123.456,
      "min_ms": 45123.456,
      "max_ms": 45123.456,
      "peak_memory_kb": 524288
    }
  ],
  "timesteps": [
    {
      "sim_time": 0.0,
      "step_duration_ms": 12.3456,
      "memory_kb": 262144
    }
  ]
}
```

---

## Adding New Instrumentation Points

To add profiling to a new section of code, follow these steps:

### Step 1: Add a New Phase to the Enum (if needed)

Edit `dynawo/sources/Solvers/Common/DYNSolverProfiler.h`:

```cpp
enum ProfilePhase {
  PHASE_SIMULATION_LOOP = 0,
  PHASE_SOLVER_SOLVE,
  // ... existing phases ...
  PHASE_IO,
  PHASE_MY_NEW_PHASE,    // <-- Add your new phase here
  PHASE_COUNT            // Must remain last
};
```

### Step 2: Add the Phase Name String

Edit `dynawo/sources/Solvers/Common/DYNSolverProfiler.cpp` in the `phaseToString()` function:

```cpp
const char* phaseToString(ProfilePhase phase) {
  switch (phase) {
    // ... existing cases ...
    case PHASE_IO:              return "IO";
    case PHASE_MY_NEW_PHASE:    return "MyNewPhase";  // <-- Add case
    default:                    return "Unknown";
  }
}
```

### Step 3: Instrument the Code

In the source file you want to profile, include the header and add the macro:

```cpp
#include "DYNSolverProfiler.h"

void MyClass::expensiveOperation() {
  DYN_PROFILE_PHASE(PHASE_MY_NEW_PHASE);   // Timer starts here

  // ... your code ...

  // Timer automatically stops when the scope exits
}
```

For memory tracking as well:

```cpp
void MyClass::memoryIntensiveOperation() {
  DYN_PROFILE_PHASE_MEM(PHASE_MY_NEW_PHASE);  // Tracks time + memory

  // ... your code ...
}
```

### Step 4: Rebuild

Rebuild Dynawo with profiling enabled (assuming `DYNAWO_CMAKE_OPTIONAL` is
already set in your `myEnvDynawo.sh` — see [Enabling Profiling](#enabling-profiling)):

```bash
./myEnvDynawo.sh clean-build-dynawo
```

The new phase will automatically appear in the profiler output.

### Using Existing Phases for Sub-Scope Measurement

If you do not need a new phase enum value, you can use an existing one to measure a sub-scope. The profiler accumulates all measurements for each phase, so multiple `PhaseTimer` instances for the same phase will add up correctly:

```cpp
void ModelMulti::evalJt(/* ... */) {
  DYN_PROFILE_PHASE(PHASE_JACOBIAN_EVAL);  // Already instrumented

  for (auto& subModel : subModels_) {
    // Each sub-model evaluation is counted toward the same phase
    subModel->evalJt(/* ... */);
  }
}
```

---

## Example Workflow

This section walks through a complete workflow from building with profiling to analyzing results.

### 1. Build with Profiling

Follow the full setup in [INSTALL_UBUNTU24.md](INSTALL_UBUNTU24.md)
(dependencies, clone, `myEnvDynawo.sh` creation, initial build), then enable
profiling and rebuild:

```bash
cd /path/to/dynawo

# Initial full build (skip if already done)
./myEnvDynawo.sh build-user

# Add profiling flags to myEnvDynawo.sh (before the envDynawo.sh call):
#   export DYNAWO_RELEASE_WITH_DEBUG=true
#   export DYNAWO_CMAKE_OPTIONAL="-DDYNAWO_PROFILING=ON"

# Clean rebuild of Dynawo only
./myEnvDynawo.sh clean-build-dynawo
```

### 2. Set Up Python Environment

```bash
cd /path/to/dynawo/performance-analysis

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Run a Profiled Simulation

```bash
export DYNAWO_PROFILING=1
cd $DYNAWO_HOME

mkdir -p results/nordic

export DYNAWO_PROFILE_OUTPUT=results/nordic/profile.json
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
```

### 4. Examine Results

The profiler prints a summary table to stdout during execution:

```
===============================================================
  DYNAWO SOLVER PERFORMANCE PROFILE
===============================================================
Phase                  Total(s)    Calls    Avg(ms)    Min(ms)    Max(ms)   Pct(%)
---------------------------------------------------------------
SimulationLoop           12.345        1  12345.0000  12345.0000  12345.0000   100.0
SolverSolve              11.890        1  11890.0000  11890.0000  11890.0000    96.3
JacobianEval              4.567      850      5.3729      2.1200     15.8900    37.0
ResidualEval              2.345     3200      0.7328      0.4100      3.2100    19.0
NRSolve                   3.210      850      3.7765      1.5600     12.3400    26.0
...
===============================================================
```

### 5. Visualize the Profile

```bash
cd /path/to/dynawo/performance-analysis
source venv/bin/activate

# Generate charts and console summary
python analyze_profile.py $DYNAWO_HOME/results/nordic/profile.csv \
    --output-dir $DYNAWO_HOME/results/nordic/charts
```

This produces `time_breakdown_pie.png`, `hotspot_bar.png`, `step_duration_ts.png`,
and `memory_usage_ts.png` in the output directory.

### 6. Detect Bottlenecks

```bash
python bottleneck_detector.py $DYNAWO_HOME/results/nordic/profile.csv
```

Review the severity-ranked findings. Based on the output:
- If JacobianEval dominates (>30%), focus on Jacobian evaluation optimizations (see OPTIMIZATION_ROADMAP.md).
- If NRSolve dominates (>25%), focus on NR tolerance tuning, Jacobian reuse, and sparse matrix optimizations.
- If ResidualEval has a very high call count relative to time steps, the Newton solver may need tuning.

### 7. Check Memory Usage

```bash
python memory_analyzer.py $DYNAWO_HOME/results/nordic/profile.csv \
    --output-dir $DYNAWO_HOME/results/nordic/memory
```

Look for potential memory leaks (growing regression slope) and high peak memory phases.

### 8. Optimize and Compare

Make optimizations, rebuild, re-run the same test case, and compare:

```bash
# Re-run after optimization
export DYNAWO_PROFILE_OUTPUT=results/nordic/profile_optimized.csv
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs

# Compare baseline vs. optimised
python compare_runs.py results/nordic/profile.csv \
                       results/nordic/profile_optimized.csv \
                       --output results/nordic/comparison.html
```

Open `comparison.html` in a browser to review per-phase speedups and any
regressions. Repeat the cycle until the target performance is reached.
