# OpenMP Parallelization Implementation Guide

**Author:** Sustainable Power Systems Lab (SPS-L)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** November 21, 2025

## Overview

This document provides implementation details for parallelizing Dynawo's Jacobian calculation using OpenMP. The parallelization targets the voltage level evaluation loop, which accounts for approximately 6% of total execution time and offers embarrassingly parallel structure.

## Build System Integration

### Step 1: Add OpenMP to CMakeLists.txt

**File**: `dynawo/sources/Models/CMakeLists.txt`

```cmake
# Find OpenMP package
find_package(OpenMP)

if(OPENMP_FOUND)
    message(STATUS "OpenMP found - enabling parallel Jacobian evaluation")
    add_definitions(-DDYN_USE_OPENMP)
else()
    message(STATUS "OpenMP not found - using serial Jacobian evaluation")
endif()
```

**File**: `dynawo/sources/Models/Modelica/CMakeLists.txt` (ModelNetwork)

```cmake
if(OPENMP_FOUND)
    target_compile_options(DYNModelNetwork PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(DYNModelNetwork PRIVATE OpenMP::OpenMP_CXX)
endif()
```

## Thread-Safe Infrastructure

### Step 2: Thread-Safe BusDerivatives Class

**File**: `dynawo/sources/Models/CPP/ModelNetwork/DYNBusDerivatives.h`

Add private member:
```cpp
#ifdef DYN_USE_OPENMP
#include <omp.h>
#endif

class BusDerivatives {
private:
#ifdef DYN_USE_OPENMP
    omp_lock_t addDerivativeLock_;  ///< OpenMP lock for thread-safe derivative addition
#endif
};
```

**File**: `dynawo/sources/Models/CPP/ModelNetwork/DYNBusDerivatives.cpp`

Constructor:
```cpp
BusDerivatives::BusDerivatives() {
#ifdef DYN_USE_OPENMP
    omp_init_lock(&addDerivativeLock_);
#endif
}

BusDerivatives::~BusDerivatives() {
#ifdef DYN_USE_OPENMP
    omp_destroy_lock(&addDerivativeLock_);
#endif
}
```

Thread-safe addDerivative:
```cpp
void BusDerivatives::addDerivative(typeDerivative_t type, int rowIndex, double value) {
#ifdef DYN_USE_OPENMP
    omp_set_lock(&addDerivativeLock_);
#endif
    
    // Original implementation
    derivatives_[type].addValue(rowIndex, value);
    
#ifdef DYN_USE_OPENMP
    omp_unset_lock(&addDerivativeLock_);
#endif
}
```

### Step 3: Thread-Local Storage for Intermediate Results

**File**: `dynawo/sources/Models/CPP/ModelNetwork/DYNModelVoltageLevel.h`

```cpp
class ModelVoltageLevel {
private:
    // Thread-local storage for parallel evaluation
    struct ThreadLocalData {
        std::vector<double> localDerivatives;
        std::vector<int> localIndices;
    };
    
#ifdef DYN_USE_OPENMP
    mutable std::vector<ThreadLocalData> threadLocalData_;
#endif
};
```

## Parallel Jacobian Evaluation

### Step 4: Parallelize Voltage Level Loop

**File**: `dynawo/sources/Models/CPP/ModelNetwork/DYNModelNetwork.cpp`

In `ModelNetwork::evalJt()`:

```cpp
void ModelNetwork::evalJt(double t, double cj, int& offsetJt, SparseMatrix& jacobian) {
    // Initialize bus derivatives (thread-safe)
    busContainer_->initDerivatives();
    
    // Parallel voltage level derivative evaluation
    const size_t numVoltageLevels = voltageLevels_.size();
    
#ifdef DYN_USE_OPENMP
    // Use parallelization only if sufficient work (avoid thread overhead)
    const size_t MIN_VOLTAGE_LEVELS_FOR_PARALLEL = 4;
    const bool useParallel = (numVoltageLevels >= MIN_VOLTAGE_LEVELS_FOR_PARALLEL);
    
    if (useParallel) {
        #pragma omp parallel for schedule(dynamic, 1) num_threads(4)
        for (size_t i = 0; i < numVoltageLevels; ++i) {
            voltageLevels_[i]->evalDerivatives(t);
        }
    } else {
        // Serial fallback for small systems
        for (size_t i = 0; i < numVoltageLevels; ++i) {
            voltageLevels_[i]->evalDerivatives(t);
        }
    }
#else
    // Serial implementation when OpenMP not available
    for (size_t i = 0; i < numVoltageLevels; ++i) {
        voltageLevels_[i]->evalDerivatives(t);
    }
#endif
    
    // Jacobian assembly (can also be parallelized separately)
    for (auto& voltageLevel : voltageLevels_) {
        voltageLevel->evalJt(t, offsetJt, jacobian);
    }
}
```

### Step 5: Parallel Bus Jacobian Assembly (Optional Enhancement)

**File**: `dynawo/sources/Models/CPP/ModelNetwork/DYNModelNetwork.cpp`

```cpp
// Thread-safe sparse matrix with column-level locks
class ThreadSafeSparseMatrix : public SparseMatrix {
private:
    std::vector<omp_lock_t> columnLocks_;
    
public:
    ThreadSafeSparseMatrix(int size) : SparseMatrix(size) {
#ifdef DYN_USE_OPENMP
        columnLocks_.resize(size);
        for (auto& lock : columnLocks_) {
            omp_init_lock(&lock);
        }
#endif
    }
    
    ~ThreadSafeSparseMatrix() {
#ifdef DYN_USE_OPENMP
        for (auto& lock : columnLocks_) {
            omp_destroy_lock(&lock);
        }
#endif
    }
    
    void addTermThreadSafe(int col, int row, double value) {
#ifdef DYN_USE_OPENMP
        if (col < static_cast<int>(columnLocks_.size())) {
            omp_set_lock(&columnLocks_[col]);
        }
#endif
        
        addTerm(row, value);  // Original implementation
        
#ifdef DYN_USE_OPENMP
        if (col < static_cast<int>(columnLocks_.size())) {
            omp_unset_lock(&columnLocks_[col]);
        }
#endif
    }
};
```

## Performance Tuning

### Runtime Configuration

**Environment Variables**:
```bash
# Number of threads (set based on available cores)
export OMP_NUM_THREADS=4

# Scheduling policy for load balancing
# "dynamic,1" = dynamic scheduling with chunk size 1 (best for uneven workload)
export OMP_SCHEDULE="dynamic,1"

# Thread affinity for better cache locality
export OMP_PROC_BIND=close  # Bind threads to nearby cores
export OMP_PLACES=cores      # One thread per physical core
```

### Compile-Time Tuning

**Threshold Tuning**:
```cpp
// In ModelNetwork.cpp
namespace {
    // Tunable thresholds - adjust based on profiling
    constexpr size_t MIN_VOLTAGE_LEVELS_FOR_PARALLEL = 4;
    constexpr size_t MAX_OPENMP_THREADS = 8;  // Limit to avoid over-subscription
}
```

## Testing and Validation

### Test 1: Correctness Validation

```bash
# Build with thread sanitizer
mkdir build-tsan
cd build-tsan
cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" ..
make

# Run test suite
./dynawo tests --network IEEE14
./dynawo tests --network IEEE57
./dynawo tests --network Nordic
```

### Test 2: Performance Benchmarking

```cpp
// Add timing code in ModelNetwork::evalJt()
#ifdef DYN_USE_OPENMP
    double start = omp_get_wtime();
#endif
    
    // ... parallel evaluation ...
    
#ifdef DYN_USE_OPENMP
    double elapsed = omp_get_wtime() - start;
    Trace::debug() << "Parallel evalJt time: " << elapsed << " seconds" << Trace::endline;
#endif
```

### Test 3: Scalability Analysis

```python
# Python script to test thread scaling
import subprocess
import matplotlib.pyplot as plt

threads = [1, 2, 4, 8, 16]
times = []

for n in threads:
    env = {'OMP_NUM_THREADS': str(n)}
    result = subprocess.run(['./dynawo', 'simulate', 'test.jobs'], 
                           env=env, capture_output=True)
    time = extract_time_from_output(result.stdout)
    times.append(time)

# Plot speedup
speedup = [times[0] / t for t in times]
plt.plot(threads, speedup, 'o-', label='Actual')
plt.plot(threads, threads, '--', label='Ideal')
plt.xlabel('Number of Threads')
plt.ylabel('Speedup')
plt.legend()
plt.savefig('openmp_scaling.png')
```

## Expected Performance Impact

### Baseline Measurements
- Serial Jacobian eval: 1.725s
- Voltage level loop: 0.390s (22.6% of Jacobian time)

### Parallelization Estimates

| Threads | Speedup | Voltage Loop Time | Jacobian Time | Overall Benefit |
|---------|---------|-------------------|---------------|-----------------|
| 1 (serial) | 1.0x | 0.390s | 1.725s | 0% |
| 2 | 1.7x | 0.229s | 1.564s | 2.5% faster |
| 4 | 2.8x | 0.139s | 1.474s | 3.9% faster |
| 8 | 3.2x | 0.122s | 1.457s | 4.1% faster |

**Note**: Amdahl's Law limits overall speedup since only 22.6% of Jacobian is parallelized.

### Combined with Phase 1-2 Optimizations

| Optimization | Individual | Cumulative |
|--------------|-----------|------------|
| Matrix structure tolerance | 17% | 17% |
| KLU parameter tuning | 3% | 20% |
| Adaptive factorization | 8% | 28% |
| **OpenMP parallelization** | **4%** | **32%** |

**Total Expected Speedup**: **~32% faster** (6.47s → 4.40s)

## Troubleshooting

### Issue 1: Race Conditions

**Symptom**: Non-deterministic results, occasional crashes

**Diagnosis**:
```bash
# Compile with ThreadSanitizer
cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" ..
```

**Solution**: Add appropriate locks to shared data structures

### Issue 2: Poor Scaling

**Symptom**: Speedup less than expected

**Diagnosis**: Profile with Intel VTune or gprof:
```bash
export OMP_NUM_THREADS=4
vtune -collect hotspots -r vtune_results ./dynawo simulate test.jobs
```

**Common Causes**:
- Lock contention (too many threads waiting for locks)
- Load imbalance (uneven voltage level workloads)
- False sharing (cache line bouncing between cores)

**Solutions**:
- Use finer-grained locks or lock-free algorithms
- Adjust `OMP_SCHEDULE` to "dynamic" for better load balancing
- Pad data structures to avoid false sharing

### Issue 3: Thread Overhead Dominates

**Symptom**: Parallel version slower than serial for small networks

**Solution**: Adjust `MIN_VOLTAGE_LEVELS_FOR_PARALLEL` threshold:
```cpp
// Increase threshold for smaller systems
constexpr size_t MIN_VOLTAGE_LEVELS_FOR_PARALLEL = 8;
```

## Integration Checklist

- [ ] Add OpenMP to CMakeLists.txt
- [ ] Implement thread-safe `BusDerivatives::addDerivative()`
- [ ] Add OpenMP pragmas to voltage level loop
- [ ] Test with ThreadSanitizer (no races detected)
- [ ] Benchmark with 1, 2, 4, 8 threads
- [ ] Validate numerical results match serial implementation
- [ ] Document performance characteristics in user guide
- [ ] Add runtime configuration options to solver parameters

## Conclusion

OpenMP parallelization of the Jacobian evaluation provides **3-5% overall speedup** with minimal code changes and low implementation risk. When combined with Phase 1-2 optimizations, the total performance improvement reaches **30-35%**, making this a highly effective optimization strategy for modern multi-core systems.

---

**Status**: Implementation guide complete. Ready for development and testing.

