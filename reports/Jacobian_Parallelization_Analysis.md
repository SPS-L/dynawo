# Jacobian Parallelization Analysis for Dynawo

**Author:** Sustainable Power Systems Lab (SPS-L)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** November 21, 2025

## Executive Summary

This document analyzes parallelization opportunities in the Dynawo Jacobian calculation pipeline, which consumes **26.7% of total execution time** (1.72504s out of 6.47s) according to Intel VTune profiling. The analysis identifies feasible parallelization strategies using OpenMP and outlines the design for partial Jacobian updates.

## Current Jacobian Evaluation Pipeline

### Call Stack Analysis

From profiling data (`LG-ar.csv`):

```
DYN::SolverKINEuler::evalJ_KIN: 1.72504s (26.7% total)
└── DYN::ModelMulti::evalJt: 1.695s (26.2%)
    └── DYN::SubModel::evalJtSub: 1.695s (26.2%)
        ├── DYN::ModelNetwork::evalJt: 1.200s (18.5%)
        │   ├── ModelVoltageLevel::evalDerivatives: 0.390s (6.0%)
        │   │   └── ModelSwitch::evalDerivatives: 0.210s (3.2%)
        │   ├── ModelVoltageLevel::evalJt: 0.330s (5.1%)
        │   │   └── ModelBus::evalJt: 0.300s (4.6%)
        │   ├── ModelBusContainer::initDerivatives: 0.300s (4.6%)
        │   ├── ModelLine::evalDerivatives: 0.150s (2.3%)
        │   └── ModelLoad::evalDerivatives: 0.030s (0.5%)
        └── DYN::ModelManager::evalJtAdept: 0.495s (7.6%)
            └── adept::Stack::jacobian_reverse: 0.420s (6.5%)
```

### Key Insights

1. **Network Model Dominates**: `ModelNetwork::evalJt` accounts for 18.5% of total time
2. **Component-Level Granularity**: Voltage levels, buses, lines, loads are independently calculable
3. **Adept AD Already Optimized**: Automatic differentiation library is already efficient
4. **Bus Derivatives Are Hotspot**: Accumulation and evaluation dominate bus-level operations

## Parallelization Opportunities

### 1. Voltage Level Parallelization ⭐⭐⭐ **High Priority**

**Target**: `ModelNetwork::evalJt` → `ModelVoltageLevel::evalDerivatives` (0.390s)

**Opportunity**:
- Multiple voltage levels in network (typically 10-100 in large systems)
- Each voltage level evaluates derivatives independently
- Minimal inter-voltage-level dependencies (only through transformers)

**Implementation Strategy**:

```cpp
// In ModelNetwork::evalJt()
#pragma omp parallel for schedule(dynamic) if(voltageLevels_.size() > 4)
for (size_t i = 0; i < voltageLevels_.size(); ++i) {
    voltageLevels_[i]->evalDerivatives(t);
}
```

**Requirements**:
- Thread-safe `BusDerivatives::addDerivative()`
- Thread-local intermediate storage
- Atomic operations or critical sections for shared bus updates

**Expected Speedup**: 2-3x on 4-core systems for 0.390s → **0.130-0.195s saved**

### 2. Sparse Matrix Assembly Parallelization ⭐⭐ **Medium Priority**

**Target**: `ModelBus::evalJt` and `SparseMatrix::addTerm` operations

**Opportunity**:
- Sparse matrix uses CSR format - column-wise parallelizable
- Each bus contributes to distinct matrix columns
- Matrix assembly is embarrassingly parallel if using thread-safe insertion

**Implementation Strategy**:

```cpp
// Thread-safe sparse matrix insertion with OpenMP locks
class SparseMatrix {
private:
    std::vector<omp_lock_t> columnLocks_;  // One lock per column
    
public:
    void addTermThreadSafe(int row, int col, double value) {
        omp_set_lock(&columnLocks_[col]);
        addTerm(row, value);  // Original implementation
        omp_unset_lock(&columnLocks_[col]);
    }
};

// Parallel bus Jacobian evaluation
#pragma omp parallel for schedule(static)
for (size_t i = 0; i < buses_.size(); ++i) {
    buses_[i]->evalJt(t, offsetJt, jacobian);  // Uses thread-safe addTerm
}
```

**Expected Speedup**: 1.5-2x for 0.300s → **0.100-0.150s saved**

### 3. Component Derivative Evaluation ⭐ **Lower Priority**

**Target**: Lines, loads, switches, transformers (0.180s combined)

**Opportunity**:
- Each component computes derivatives independently
- Large networks have 1000+ components

**Challenges**:
- Overhead of thread creation may exceed benefit for small components
- Better suited for coarse-grained parallelization (at voltage level)

**Implementation**: Covered by voltage level parallelization

## Partial Jacobian Update Design

### Motivation

Current implementation recalculates **entire Jacobian** on every mode change or discrete event, even when only a small subset of equations/variables changed.

**Problem Analysis from Profiling**:
- Mode changes occur at t=1,3,4,5,6,7 (6 times in 7-second simulation)
- Each triggers full Jacobian recalculation (0.246s average)
- If only 10% of system changes, 90% of work is wasted

### Architecture Design

#### 1. Change Tracking Infrastructure

```cpp
class JacobianChangeTracker {
public:
    // Track which sub-models have changed states
    void markSubModelChanged(int subModelId);
    
    // Track which variable ranges are affected
    void markVariableRangeChanged(int startIdx, int endIdx);
    
    // Query if a block needs recalculation
    bool needsUpdate(int blockId) const;
    
    // Reset after full Jacobian update
    void reset();
    
private:
    std::vector<bool> changedSubModels_;
    std::vector<std::pair<int, int>> changedRanges_;
    bool fullUpdateRequired_;
};
```

#### 2. Block-Structured Jacobian

```cpp
class BlockJacobian {
public:
    struct Block {
        int rowStart, rowEnd;
        int colStart, colEnd;
        int subModelId;
        SparseMatrix localMatrix;
    };
    
    // Update only changed blocks
    void updateBlocks(const std::vector<int>& blockIds, double t);
    
    // Merge updated blocks into global Jacobian
    void mergeBlocks();
    
private:
    std::vector<Block> blocks_;
    SparseMatrix globalJacobian_;
};
```

#### 3. Selective Update Strategy

```cpp
// In ModelMulti::evalJt()
void ModelMulti::evalJt(double t, double cj, SparseMatrix& jt) {
    if (changeTracker_.isFullUpdateRequired()) {
        // Full Jacobian recalculation
        evalJtFull(t, cj, jt);
        changeTracker_.reset();
    } else {
        // Partial update - only changed blocks
        std::vector<int> changedBlocks = changeTracker_.getChangedBlocks();
        
        for (int blockId : changedBlocks) {
            subModels_[blockId]->evalJtSub(t, cj, offsetJt, jt);
        }
        
        changeTracker_.reset();
    }
}
```

#### 4. Integration with Mode Changes

```cpp
// In Model::evalG() - after mode change detection
void Model::notifyModeChange(int subModelId, modeChangeType_t changeType) {
    if (changeType == ALGEBRAIC_J_UPDATE_MODE) {
        // Mark affected sub-model and dependencies
        jacobianChangeTracker_.markSubModelChanged(subModelId);
        
        // Propagate to dependent sub-models (e.g., connected buses)
        for (int depId : getDependentSubModels(subModelId)) {
            jacobianChangeTracker_.markSubModelChanged(depId);
        }
    }
}
```

### Expected Impact

**Conservative Estimate**:
- 40% of mode changes affect <20% of system
- Partial update saves: 0.80 × 0.246s × 6 events = **0.71s saved**

**Optimistic Estimate** (isolated component changes):
- 60% of mode changes affect <10% of system  
- Partial update saves: 0.90 × 0.246s × 6 events = **1.33s saved**

### Implementation Challenges

1. **Dependency Tracking**: Complex graph of interconnected components
2. **Sparse Matrix Updates**: Efficient in-place updates to CSR format
3. **KLU Integration**: May still require full symbolic factorization
4. **Testing Complexity**: Ensure correctness with partial updates

## Implementation Recommendations

### Phase 1: OpenMP Voltage Level Parallelization (Weeks 9-10)

**Priority**: HIGH - Best effort/reward ratio

**Steps**:
1. Add OpenMP flags to CMakeLists.txt
2. Implement thread-safe `BusDerivatives::addDerivative()`
3. Parallelize voltage level loop in `ModelNetwork::evalJt()`
4. Extensive testing with thread sanitizers
5. Performance benchmarking with 2, 4, 8 threads

**Risk**: LOW - Well-understood parallelization pattern

**Expected Benefit**: 10-15% overall speedup

### Phase 2: Sparse Matrix Thread Safety (Week 10)

**Priority**: MEDIUM - Enables broader parallelization

**Steps**:
1. Implement column-level locks in `SparseMatrix`
2. Create thread-safe `addTerm()` variant
3. Update bus/component Jacobian contributions
4. Validate matrix correctness with reference solutions

**Risk**: MEDIUM - Lock contention could reduce benefits

**Expected Benefit**: Additional 5-8% speedup

### Phase 3: Partial Jacobian Updates (Week 11)

**Priority**: MEDIUM - High complexity, high reward

**Steps**:
1. Design and implement `JacobianChangeTracker`
2. Add change notification to mode change handlers
3. Implement selective update logic in `ModelMulti::evalJt()`
4. Comprehensive testing of correctness
5. Benchmark on scenarios with frequent mode changes

**Risk**: HIGH - Complex dependencies, potential for subtle bugs

**Expected Benefit**: 5-10% speedup for mode-change-heavy scenarios

## Performance Metrics

### Target Measurements

| Metric | Baseline | Target | Measurement |
|--------|----------|--------|-------------|
| Jacobian eval time | 1.725s | <1.200s | VTune profiling |
| Parallel efficiency | N/A | >60% | Speedup/(num_threads) |
| Thread scaling | N/A | Linear to 4 threads | Benchmark suite |
| Partial update ratio | 100% | <40% | Change tracker stats |

### Validation Criteria

1. **Correctness**: Solution accuracy within 1e-8 of serial implementation
2. **Performance**: Minimum 15% overall speedup (0.25s reduction)
3. **Scalability**: Near-linear speedup up to 4 threads
4. **Robustness**: No race conditions (ThreadSanitizer clean)
5. **Maintainability**: Clear code with minimal complexity increase

## OpenMP Configuration

### Compiler Flags

```cmake
# CMakeLists.txt additions
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    target_link_libraries(dynawo_ModelNetwork OpenMP::OpenMP_CXX)
endif()
```

### Runtime Environment Variables

```bash
# Optimal settings for power system simulations (tuned experimentally)
export OMP_NUM_THREADS=4           # Use 4 threads (typical server)
export OMP_SCHEDULE="dynamic,1"    # Dynamic scheduling for load balance
export OMP_PROC_BIND=true          # Bind threads to cores
export OMP_PLACES=cores            # One thread per core
```

## Risk Assessment

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Race conditions | Medium | High | ThreadSanitizer, extensive testing |
| Lock contention overhead | High | Medium | Fine-grained locks, lock-free algorithms |
| Non-deterministic results | Low | High | Reproducible summation, atomic operations |
| Limited scalability | Medium | Medium | Profile-guided optimization, better decomposition |
| Integration complexity | High | High | Incremental development, feature flags |

### Schedule Risks

- **OpenMP parallelization**: 2 weeks (achievable)
- **Thread-safe sparse matrix**: 1 week (achievable)
- **Partial Jacobian updates**: 2-3 weeks (aggressive)

**Contingency**: If partial updates prove too complex, focus on OpenMP parallelization which delivers 60-70% of potential benefit with lower risk.

## Conclusion

Jacobian parallelization offers **15-25% overall simulation speedup** with manageable implementation complexity. Voltage level parallelization should be prioritized as Phase 1, with sparse matrix thread safety and partial updates as follow-on enhancements.

The combination of:
- **Phase 1-2 optimizations** (structure tolerance, KLU tuning, adaptive factorization): 15-24% speedup
- **Phase 3 parallelization** (OpenMP, partial updates): 15-25% additional speedup
- **Combined total**: **30-40% overall speedup** potential

This positions Dynawo for significantly improved performance on modern multi-core systems while maintaining numerical accuracy and code maintainability.

---

**Next Steps**:
1. Add OpenMP support to build system
2. Implement prototype voltage level parallelization
3. Validate correctness and measure performance
4. Iterate based on profiling results
5. Document best practices and configuration guidelines

