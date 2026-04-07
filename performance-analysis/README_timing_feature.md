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
- **computation_time_ms**: Timestep computation time in milliseconds (3 decimal places)
- **accumulated_computation_time_s**: Total accumulated computation time from simulation start in seconds (3 decimal places)

### Understanding Accumulated Time

The **accumulated_computation_time_s** column provides the total computational cost from the beginning of the simulation until each measurement point:

- **Scope**: Excludes model compilation, initialization, and setup phases
- **Start Point**: Begins when the main simulation loop starts executing timesteps
- **Purpose**: Shows cumulative computational expense for performance analysis
- **Use Cases**:
  - Track total computational resources used over simulation duration
  - Identify performance trends and optimization opportunities
  - Compare computational efficiency across different simulation scenarios

**Example**: If the first timestep takes 0.001s, the accumulated time will be 0.001s. If the second timestep takes 0.002s, the accumulated time becomes 0.003s, and so on.

## Performance Impact

- **Zero overhead when disabled**: No performance impact when timing is not enabled
- **Minimal overhead when enabled**: High-precision timing with pre-allocated memory
- **Automatic memory optimization**: Vector pre-allocation based on estimated simulation length

## Test Files

- `test_timing_job.jobs` - Example with timing enabled
- `test_timing_disabled.jobs` - Example with timing explicitly disabled  
- `test_timing_default.jobs` - Example with default behavior (no attribute)
- `test_timing_output/simRT.csv` - Example output file

## Implementation Details

- Uses C++ `std::chrono::high_resolution_clock` for maximum precision
- Measures time from solver start to timestep completion
- **Accumulated timing excludes initialization phases** - starts when entering the main simulation loop
- **Total elapsed time tracking** - provides cumulative computation cost from simulation start
- Integrates with existing Dynawo error handling and file management
- Follows Dynawo XML schema validation patterns
- Maintains backward compatibility with existing CSV parsers (first two columns unchanged)
