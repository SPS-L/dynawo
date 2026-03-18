# Dynawo Performance Analysis Framework

A comprehensive profiling and benchmarking framework for the Dynawo power system simulation tool. This framework instruments the solver pipeline, collects per-phase timing and memory data, and provides Python-based analysis tools for visualization and regression detection.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Components](#components)
3. [Directory Structure](#directory-structure)
4. [Enabling Profiling](#enabling-profiling)
5. [Running Benchmarks](#running-benchmarks)
6. [Interpreting Results](#interpreting-results)
7. [Adding New Instrumentation Points](#adding-new-instrumentation-points)
8. [Example Workflow](#example-workflow)

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

**Benchmark Layer:** Shell scripts and solver configuration files automate running standardized test cases (IEEE 14-bus, IEEE 39-bus, large-scale networks) with consistent solver settings, collecting profiling data across runs for comparison.

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
  - `PHASE_LINEAR_SOLVE` -- sparse linear system solve (KLU)
  - `PHASE_MATRIX_COPY` -- matrix data copy operations
  - `PHASE_KINSOL_SOLVE` -- KINSOL nonlinear solve
  - `PHASE_REINIT` -- solver reinitialization after events
  - `PHASE_IO` -- file I/O operations

- **`PhaseStats` struct:** Stores per-phase statistics (total time, min/max/avg call time, call count, peak memory).

- **`SolverProfiler` class (singleton):** Central data collector. Provides `record()`, `recordWithMemory()`, and `recordTimestep()` methods. Exports to CSV or JSON via `exportCSV()` and `exportJSON()`. Automatically exports on destruction if `DYNAWO_PROFILE_OUTPUT` environment variable is set.

- **`PhaseTimer` class (RAII):** Construct at the top of a scope to automatically measure its duration. The destructor records the elapsed time to the global profiler. Optionally tracks memory via `/proc/self/status`.

- **Profiling macros:**
  - `DYN_PROFILE_PHASE(phase)` -- create a scoped timer for a phase
  - `DYN_PROFILE_PHASE_MEM(phase)` -- scoped timer with memory tracking
  - `DYN_PROFILE_RECORD_TIMESTEP(simTime, stepMs, memKB)` -- record a timestep entry
  - `DYN_PROFILE_PRINT_REPORT()` -- print summary to stdout
  - `DYN_PROFILE_EXPORT_CSV(filename)` -- export CSV
  - `DYN_PROFILE_EXPORT_JSON(filename)` -- export JSON

### Python Analysis Tools

Located in `performance-analysis/`:

- **`requirements.txt`:** Python package dependencies (matplotlib, pandas, numpy, jinja2).

### Benchmark Infrastructure

Located in `performance-analysis/benchmarks/`:

- **`solver_configs/`:** Predefined solver parameter files for consistent benchmarking across different solver configurations (IDA parameters, time step settings, tolerance levels).

---

## Directory Structure

```
performance-analysis/
|-- README.md                    # This file
|-- OPTIMIZATION_ROADMAP.md      # Optimization suggestions and roadmap
|-- requirements.txt             # Python dependencies
|-- benchmarks/
|   |-- solver_configs/          # Predefined solver configurations
|   |   |-- (IDA configs, SIM configs, tolerance presets)

dynawo/sources/Solvers/Common/
|-- DYNSolverProfiler.h          # Profiler header (enums, classes, macros)
|-- DYNSolverProfiler.cpp        # Profiler implementation (stats, export)
```

---

## Enabling Profiling

### Method 1: Build-Time Activation (Recommended)

Add the CMake flag when configuring the build:

```bash
cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=Release \
    -DDYNAWO_PROFILING=ON \
    # ... other flags ...
```

This defines the `DYNAWO_PROFILING` preprocessor macro, which activates all `DYN_PROFILE_PHASE` and related macros throughout the solver code. Without this flag, these macros expand to `((void)0)` and are optimized away completely.

### Method 2: Runtime Activation

Set the environment variable before running the simulation:

```bash
export DYNAWO_PROFILING=1
```

The `SolverProfiler` constructor checks for this variable. This enables the singleton's `record()` and `recordTimestep()` methods, but the RAII `PhaseTimer` macros will only be active if the build was compiled with `-DDYNAWO_PROFILING=ON`.

For full instrumentation, use both methods together.

### Automatic Export

Set `DYNAWO_PROFILE_OUTPUT` to automatically write profiling data when the simulation finishes:

```bash
export DYNAWO_PROFILE_OUTPUT=/path/to/results/profile.csv
# or for JSON:
export DYNAWO_PROFILE_OUTPUT=/path/to/results/profile.json
```

The file extension determines the format. The profiler writes data in its destructor at process exit.

---

## Running Benchmarks

### Basic Benchmark Run

```bash
# 1. Set up the environment
export DYNAWO_PROFILING=1
export DYNAWO_HOME=$HOME/dynawo-install

# 2. Create results directory
mkdir -p results/ieee14

# 3. Run with profiling export
export DYNAWO_PROFILE_OUTPUT=results/ieee14/profile.csv
$DYNAWO_HOME/bin/dynawo jobs --input examples/IEEE14/IEEE14.jobs

# 4. View the summary (printed to stdout during execution)
```

### Using Solver Configurations

The `benchmarks/solver_configs/` directory contains predefined solver parameter files that can be used for consistent benchmarking:

```bash
# Run with a specific solver configuration
$DYNAWO_HOME/bin/dynawo jobs \
    --input examples/IEEE14/IEEE14.jobs \
    --solver-config benchmarks/solver_configs/ida_default.par
```

### Multiple-Run Benchmarking

For statistical significance, run each benchmark multiple times:

```bash
for i in $(seq 1 5); do
    export DYNAWO_PROFILE_OUTPUT=results/ieee14/run_${i}.csv
    $DYNAWO_HOME/bin/dynawo jobs --input examples/IEEE14/IEEE14.jobs
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

- **LinearSolve:** Sparse linear system solve using KLU. This is the other major bottleneck alongside JacobianEval. High percentage here (>20%) indicates the linear algebra is dominant.

- **MatrixCopy:** Copying matrix data between internal representations. Should be a small fraction; if not, this indicates unnecessary data movement.

- **KINSOLSolve:** KINSOL nonlinear solver calls (used for initial condition calculation and event reinitialization).

- **Reinit:** Solver reinitialization after mode changes or events. Frequent reinitializations indicate many discrete events.

- **IO:** File I/O operations (reading input, writing output files). Should be small unless the output frequency is very high.

### Key Metrics to Watch

1. **JacobianEval + LinearSolve combined percentage:** If these together exceed 60% of simulation time, the simulation is compute-bound on sparse linear algebra. Consider the optimization strategies in `OPTIMIZATION_ROADMAP.md`.

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

Rebuild with profiling enabled:

```bash
cmake ../dynawo -DDYNAWO_PROFILING=ON
make -j$(nproc)
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

```bash
cd /path/to/dynawo
mkdir -p build && cd build

cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DDYNAWO_PROFILING=ON \
    -DDYNAWO_HOME=$(pwd)/.. \
    -DCMAKE_INSTALL_PREFIX=$HOME/dynawo-install \
    -DINSTALL_OPENMODELICA=$HOME/OpenModelica

make -j$(nproc)
make install
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
export DYNAWO_HOME=$HOME/dynawo-install

mkdir -p results/ieee14

export DYNAWO_PROFILE_OUTPUT=results/ieee14/profile.json
$DYNAWO_HOME/bin/dynawo jobs --input examples/IEEE14/IEEE14.jobs
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
LinearSolve               3.210      850      3.7765      1.5600     12.3400    26.0
...
===============================================================
```

### 5. Analyze and Optimize

Based on the output:
- If JacobianEval dominates (>30%), focus on Jacobian evaluation optimizations (see OPTIMIZATION_ROADMAP.md).
- If LinearSolve dominates (>25%), focus on KLU and sparse matrix optimizations.
- If ResidualEval has a very high call count relative to time steps, the Newton solver may need tuning.
- Compare results across different solver configurations using the benchmark infrastructure.

### 6. Iterate

Make optimizations, rebuild, re-run the same test case, and compare the profiling data to verify improvements:

```bash
# After optimization
export DYNAWO_PROFILE_OUTPUT=results/ieee14/profile_optimized.json
$DYNAWO_HOME/bin/dynawo jobs --input examples/IEEE14/IEEE14.jobs

# Compare the two JSON files to see improvements
```
