# Real-Time Tracking Feature

This document describes the real-time tracking feature that allows measuring and exporting the computation time for each simulation timestep.

## Overview

The real-time tracking feature provides high-precision timing measurements of simulation timestep computation times, exported as a CSV file for analysis and performance monitoring.

## Configuration

Add the `enableRealTimeTracking` attribute to the simulation element in your job file:

```xml
<dyn:simulation startTime="0" stopTime="30" enableRealTimeTracking="true"/>
```

### Attribute Options:
- `enableRealTimeTracking="true"` - Enable timing measurement and CSV export
- `enableRealTimeTracking="false"` - Disable timing measurement (explicit)
- No attribute - Default behavior (timing disabled)

## Output

When enabled, the feature creates a `simRT.csv` file in the simulation outputs directory with the following format:

```csv
simulation_time,computation_time_ms,accumulated_computation_time_s
0.000000,0.125,0.000
0.001000,0.089,0.001
0.002000,0.092,0.002
0.003000,0.095,0.003
```

### CSV Format:
- **simulation_time**: Current simulation time (6 decimal places)
- **computation_time_ms**: Timestep computation time in milliseconds (3 decimal places), measured from just before `solver_->solve()` to just after `model_->notifyTimeStep()`
- **accumulated_computation_time_s**: Wall-clock time elapsed since the main simulation loop started, in seconds (3 decimal places)

### Understanding Accumulated Time

The **accumulated_computation_time_s** column is the wall-clock time since the main loop began — **not** the running sum of the `computation_time_ms` column:

- **Scope**: Excludes model compilation, initialization, and setup phases
- **Start Point**: Begins when the main simulation loop starts executing timesteps
- **Caveat**: It also includes per-iteration work outside the per-step window — intermediate state/IIDM dumps, the timeout check, and (in profiling builds) the profiler's own per-step record. On dump-heavy runs, `sum(computation_time_ms)` and this column diverge systematically; use this column for RTR (real-time ratio) calculations, and column 2 for per-step analysis.

## Performance Impact

- **Zero overhead when disabled**: No performance impact when timing is not enabled
- **Minimal overhead when enabled**: High-precision timing with pre-allocated memory
- **Automatic memory optimization**: Vector pre-allocation based on estimated simulation length

## Implementation Details

- Implemented in `dynawo/sources/Simulation/DYNSimulation.cpp` (timing capture in the main loop, export in `writeRealTimeTrackingFile()` called from `terminate()`); the `.jobs` attribute is declared in `dynawo/sources/API/JOB/xsd/jobs.xsd`
- Uses C++ `std::chrono::high_resolution_clock` (note: on libstdc++ this aliases the non-monotonic `system_clock`; NTP adjustments during long runs can skew results)
- Measures time from solver start to timestep completion
- **Accumulated timing excludes initialization phases** - starts when entering the main simulation loop
- Jobs files carrying the attribute fail XSD validation on stock upstream Dynawo (the attribute only exists in this fork's schema)
- `SimulationRT` (the real-time co-simulation loop) does **not** produce `simRT.csv`; the feature applies to the standard `Simulation` loop only
- Maintains backward compatibility with existing CSV parsers (first two columns unchanged)
