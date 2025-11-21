# Partial Jacobian Update Mechanism Design

**Author:** Sustainable Power Systems Lab (SPS-L)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** November 21, 2025

## Executive Summary

This document presents a detailed design for partial Jacobian updates in Dynawo, enabling selective recalculation of only changed matrix blocks during mode changes and discrete events. Analysis shows potential **10-20% speedup** for scenarios with frequent mode changes by avoiding unnecessary recalculations of unchanged system components.

## Problem Statement

### Current Behavior

**Full Jacobian Recalculation on Every Mode Change**:
- Mode changes occur 6 times in 7-second PFR simulation
- Each triggers complete Jacobian recalculation (~0.246s)
- Total mode-change-related Jacobian time: 6 × 0.246s = **1.476s**

### Inefficiency Analysis

Most mode changes affect only localized portions of the system:

| Event Type | Affected % | Example |
|------------|-----------|---------|
| Generator trip | 5-10% | Generator + directly connected buses |
| Load switching | 1-5% | Load + connection bus |
| Transformer tap change | 10-20% | Transformer + connected buses |
| Line opening | 15-30% | Line + terminal buses + flows |
| HVDC control mode | 10-20% | HVDC + AC connection points |

**Opportunity**: If only 15% of system changes, **85% of Jacobian recalculation is wasted work**.

## Design Architecture

### 1. Change Tracking System

#### Component-Level Granularity

```cpp
/**
 * @brief Tracks which components and variables have changed since last Jacobian update
 * 
 * Maintains fine-grained change information to enable selective Jacobian updates.
 * Integrated with mode change detection and state variable modifications.
 */
class JacobianChangeTracker {
public:
    /**
     * @brief Mark a sub-model as having changed state
     * @param subModelId unique identifier of the sub-model
     */
    void markSubModelChanged(int subModelId);
    
    /**
     * @brief Mark a range of variables as having changed
     * @param startIdx starting variable index
     * @param endIdx ending variable index (inclusive)
     */
    void markVariableRangeChanged(int startIdx, int endIdx);
    
    /**
     * @brief Mark a specific component as having changed
     * @param componentType type of component (bus, line, generator, etc.)
     * @param componentId unique component identifier
     */
    void markComponentChanged(ComponentType componentType, int componentId);
    
    /**
     * @brief Propagate changes to dependent components
     * 
     * When a component changes, its electrically connected neighbors
     * must also be marked as affected (e.g., bus connected to changed line).
     */
    void propagateDependencies();
    
    /**
     * @brief Get list of sub-models requiring Jacobian update
     * @return vector of sub-model IDs that need recalculation
     */
    std::vector<int> getChangedSubModels() const;
    
    /**
     * @brief Check if a full Jacobian update is required
     * @return true if too many components changed (exceeds threshold)
     */
    bool requiresFullUpdate() const;
    
    /**
     * @brief Reset tracker after Jacobian update
     */
    void reset();
    
    /**
     * @brief Force a full update on next Jacobian evaluation
     */
    void forceFullUpdate();
    
private:
    std::set<int> changedSubModels_;                    ///< Sub-models with changes
    std::set<int> changedComponents_;                   ///< Changed component IDs
    std::map<int, std::set<int>> componentDependencies_; ///< Dependency graph
    bool fullUpdateRequired_;                            ///< Force full update flag
    
    // Thresholds
    static constexpr double FULL_UPDATE_THRESHOLD = 0.30;  ///< If >30% changed, do full update
};
```

#### Dependency Graph Construction

```cpp
/**
 * @brief Build dependency graph between components
 * 
 * Called during model initialization to establish which components
 * affect each other's Jacobian contributions.
 */
class DependencyGraphBuilder {
public:
    /**
     * @brief Construct dependency graph from network topology
     * @param model the power system model
     * @return dependency map (component -> dependent components)
     */
    static std::map<int, std::set<int>> buildGraph(const Model& model);
    
private:
    /**
     * @brief Add bus-to-branch dependencies
     * 
     * When a bus changes, all connected branches are affected.
     */
    static void addBusDependencies(const ModelBus& bus, 
                                   std::map<int, std::set<int>>& graph);
    
    /**
     * @brief Add branch-to-bus dependencies
     * 
     * When a branch changes, its terminal buses are affected.
     */
    static void addBranchDependencies(const ModelLine& line,
                                      std::map<int, std::set<int>>& graph);
};
```

### 2. Block-Structured Jacobian

#### Jacobian Block Representation

```cpp
/**
 * @brief Represents a block of the Jacobian matrix
 * 
 * Each block corresponds to a sub-model's contribution to the global Jacobian.
 * Blocks can be independently computed and merged into the global matrix.
 */
struct JacobianBlock {
    int blockId;              ///< Unique block identifier
    int rowStart;             ///< Starting row index in global Jacobian
    int rowEnd;               ///< Ending row index (exclusive)
    int colStart;             ///< Starting column index
    int colEnd;               ///< Ending column index
    int subModelId;           ///< Associated sub-model
    
    SparseMatrix localMatrix; ///< Local sparse matrix for this block
    bool isDirty;             ///< True if block needs recalculation
    
    /**
     * @brief Compute this block's Jacobian contribution
     * @param model the power system model
     * @param t current simulation time
     * @param cj Jacobian coefficient (for DAE systems)
     */
    void compute(const Model& model, double t, double cj);
};

/**
 * @brief Manages block-structured Jacobian matrix
 */
class BlockJacobian {
public:
    /**
     * @brief Initialize blocks from model structure
     * @param model the power system model
     */
    void initialize(const Model& model);
    
    /**
     * @brief Update only dirty (changed) blocks
     * @param t current simulation time
     * @param cj Jacobian coefficient
     */
    void updateDirtyBlocks(double t, double cj);
    
    /**
     * @brief Merge all blocks into global Jacobian
     * @param globalJacobian output sparse matrix
     */
    void mergeIntoGlobal(SparseMatrix& globalJacobian);
    
    /**
     * @brief Mark specific blocks as dirty based on change tracker
     * @param changeTracker change tracking information
     */
    void markDirtyBlocks(const JacobianChangeTracker& changeTracker);
    
    /**
     * @brief Get number of blocks requiring update
     * @return count of dirty blocks
     */
    int getDirtyBlockCount() const;
    
private:
    std::vector<JacobianBlock> blocks_;  ///< All Jacobian blocks
    SparseMatrix globalMatrix_;          ///< Merged global Jacobian
};
```

### 3. Selective Update Strategy

#### Update Decision Logic

```cpp
/**
 * @brief Determine whether to perform full or partial Jacobian update
 */
enum class JacobianUpdateStrategy {
    FULL_UPDATE,      ///< Recalculate entire Jacobian
    PARTIAL_UPDATE,   ///< Update only changed blocks
    NO_UPDATE         ///< Reuse previous Jacobian (if safe)
};

/**
 * @brief Decide Jacobian update strategy based on system state
 */
class JacobianUpdateDecider {
public:
    /**
     * @brief Determine optimal update strategy
     * 
     * @param changeTracker information about what changed
     * @param lastUpdateTime time since last Jacobian update
     * @param convergenceHistory recent convergence performance
     * @return recommended update strategy
     */
    static JacobianUpdateStrategy decideStrategy(
        const JacobianChangeTracker& changeTracker,
        double lastUpdateTime,
        const ConvergenceHistory& convergenceHistory
    );
    
private:
    // Decision thresholds
    static constexpr double MAX_PARTIAL_UPDATE_FRACTION = 0.30;  ///< Max 30% of blocks
    static constexpr double MAX_TIME_WITHOUT_UPDATE = 5.0;       ///< Max 5 seconds
    static constexpr int MIN_CONVERGE_ITERS_FOR_REUSE = 3;       ///< Min Newton iters
};
```

#### Integration with evalJt()

```cpp
/**
 * @brief Modified Jacobian evaluation with partial update support
 * 
 * File: ModelMulti.cpp
 */
void ModelMulti::evalJt(double t, double cj, SparseMatrix& jt) {
    // Determine update strategy
    JacobianUpdateStrategy strategy = jacUpdateDecider_.decideStrategy(
        changeTracker_, t - lastJacobianUpdateTime_, convergenceHistory_
    );
    
    switch (strategy) {
        case JacobianUpdateStrategy::FULL_UPDATE:
            // Traditional full recalculation
            evalJtFull(t, cj, jt);
            changeTracker_.reset();
            lastJacobianUpdateTime_ = t;
            stats_.fullJacobianUpdates_++;
            break;
            
        case JacobianUpdateStrategy::PARTIAL_UPDATE:
            // Selective block update
            blockJacobian_.markDirtyBlocks(changeTracker_);
            blockJacobian_.updateDirtyBlocks(t, cj);
            blockJacobian_.mergeIntoGlobal(jt);
            changeTracker_.reset();
            lastJacobianUpdateTime_ = t;
            stats_.partialJacobianUpdates_++;
            Trace::debug() << "Partial Jacobian: updated " 
                          << blockJacobian_.getDirtyBlockCount() 
                          << " blocks" << Trace::endline;
            break;
            
        case JacobianUpdateStrategy::NO_UPDATE:
            // Reuse previous Jacobian (risky - only if convergence is excellent)
            stats_.jacobianReuses_++;
            break;
    }
}
```

### 4. Mode Change Integration

#### Notification Mechanism

```cpp
/**
 * @brief Notify Jacobian system of mode changes
 * 
 * File: Model.cpp - Called when discrete variable changes detected
 */
void Model::notifyModeChange(modeChangeType_t changeType) {
    // Identify which sub-models changed
    std::vector<int> changedSubModels = detectChangedSubModels();
    
    for (int subModelId : changedSubModels) {
        jacobianChangeTracker_.markSubModelChanged(subModelId);
    }
    
    // Propagate to dependent sub-models
    jacobianChangeTracker_.propagateDependencies();
    
    // If major mode change, force factorization
    if (changeType == ALGEBRAIC_J_UPDATE_MODE) {
        factorizationForced_ = true;
    }
}

/**
 * @brief Detect which sub-models experienced mode changes
 */
std::vector<int> Model::detectChangedSubModels() {
    std::vector<int> changed;
    
    for (size_t i = 0; i < subModels_.size(); ++i) {
        if (subModels_[i]->modeChanged()) {
            changed.push_back(i);
        }
    }
    
    return changed;
}
```

## Implementation Considerations

### Sparse Matrix Update Efficiency

**Challenge**: Efficiently updating CSR sparse matrix in-place

**Solution 1 - Block Assembly**:
```cpp
// Assemble each block independently, then merge
for (const auto& block : dirtyBlocks) {
    SparseMatrix localJac;
    block.compute(model, t, cj, localJac);
    globalJacobian.mergeBlock(block.rowStart, block.colStart, localJac);
}
```

**Solution 2 - Incremental CSR Update**:
```cpp
// Update CSR arrays directly (complex but efficient)
void SparseMatrix::updateSubmatrix(int rowStart, int rowEnd, 
                                   const SparseMatrix& submatrix) {
    // Find insertion points in Ap_, Ai_, Ax_ arrays
    // Update in-place without full reallocation
    // Requires careful bookkeeping of NNZ per row
}
```

### KLU Factorization Impact

**Issue**: Partial Jacobian updates may still require full symbolic factorization

**Mitigation**:
1. **Sparsity Pattern Preservation**: If block updates don't change sparsity, only numerical factorization needed
2. **Lazy Factorization**: Defer factorization until truly necessary (next Newton iteration)
3. **Structure Change Tolerance**: Use tolerance from Phase 1 to avoid unnecessary refactorizations

### Correctness Validation

**Testing Strategy**:

```cpp
// Validation test: compare partial vs full update
void testPartialJacobianCorrectness() {
    Model model = loadTestCase("IEEE14");
    
    // Compute full Jacobian
    SparseMatrix fullJac;
    model.evalJt(t, cj, fullJac);
    
    // Simulate mode change
    model.changeGeneratorMode(genId, newMode);
    
    // Compute partial update
    model.enablePartialJacobianUpdates(true);
    SparseMatrix partialJac;
    model.evalJt(t, cj, partialJac);
    
    // Compute reference (full update)
    model.enablePartialJacobianUpdates(false);
    SparseMatrix refJac;
    model.evalJt(t, cj, refJac);
    
    // Compare
    double maxDiff = computeMaxDifference(partialJac, refJac);
    assert(maxDiff < 1e-10);  // Should be identical
}
```

## Performance Analysis

### Expected Performance Gains

**Scenario 1: Small Localized Changes (60% of events)**
- Changed components: 10%
- Partial update saves: 90% of Jacobian time
- Per-event savings: 0.90 × 0.246s = 0.221s
- Total savings: 0.60 × 6 events × 0.221s = **0.80s**

**Scenario 2: Medium Changes (30% of events)**
- Changed components: 25%
- Partial update saves: 75% of Jacobian time
- Per-event savings: 0.75 × 0.246s = 0.185s
- Total savings: 0.30 × 6 events × 0.185s = **0.33s**

**Scenario 3: Large Changes (10% of events)**
- Changed components: >30% → trigger full update
- No savings

**Total Expected Savings**: 0.80s + 0.33s = **1.13s** (17.5% overall speedup)

### Overhead Analysis

**Additional Overhead**:
- Change tracking: ~0.001s per mode change
- Dependency propagation: ~0.002s per change
- Block merging: ~0.005-0.010s per partial update

**Total Overhead**: ~6 × 0.013s = **0.078s**

**Net Benefit**: 1.13s - 0.078s = **1.05s saved** (16.2% speedup)

## Implementation Roadmap

### Phase 1: Infrastructure (Week 11.1-11.2)
- Implement `JacobianChangeTracker` class
- Build dependency graph from network topology
- Add change notification to mode change handlers

### Phase 2: Block Jacobian (Week 11.3-11.4)
- Design `JacobianBlock` structure
- Implement block computation and merging
- Test with simple network (IEEE14)

### Phase 3: Integration (Week 11.5)
- Integrate with `Model::evalJt()`
- Add update strategy decision logic
- Comprehensive testing

### Phase 4: Optimization and Validation (Week 12)
- Profile and optimize block merging
- Validate correctness on all test networks
- Measure performance improvements

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Incorrect dependency tracking | Medium | High | Extensive unit tests, reference validation |
| Sparse matrix corruption | Low | High | Incremental testing, checksum validation |
| Marginal performance gains | Medium | Medium | Early prototyping, bail-out to full updates |
| Complex debugging | High | Medium | Detailed logging, visualization tools |
| Maintenance burden | High | Medium | Clean abstractions, comprehensive documentation |

## Conclusion

Partial Jacobian updates offer **10-20% speedup potential** for mode-change-heavy scenarios with manageable implementation complexity. The design provides clean abstractions and fail-safe fallbacks to full updates, making it a low-risk enhancement with significant performance benefits.

**Recommendation**: Implement as Phase 3 enhancement after OpenMP parallelization, with careful validation and performance monitoring.

---

**Status**: Design complete. Ready for prototyping and validation.

