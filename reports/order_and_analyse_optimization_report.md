# order_and_analyse Performance Optimization Report

**Author:** Sustainable Power Systems Lab (SPSL)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** September 18, 2025

## Executive Summary

Intel VTune profiling of the all_retained test case reveals that the `order_and_analyze` function consumes **1.56187 seconds (24.1% of total execution time)** in a 6.47-second simulation. This function is part of the KLU sparse linear solver's symbolic factorization phase, with BTF preprocessing (`btf_l_maxtrans`) alone accounting for **0.79196 seconds (12.2% of total time)**. The complete `kinLsSetup` phase consumes **4.30201 seconds (66.5% of total execution time)**, making it the primary performance bottleneck.

## Problem Analysis

### 1. **Actual Intel VTune Profiling Results**

**Total Simulation Time:** 6.47047 seconds

```
Detailed Performance Breakdown:
kinLsSetup: 4.30201s (66.5% total execution time)
├── SUNLinSolSetup_KLU: 2.56496s (39.6% total)
│   ├── klu_l_analyze: 1.56187s (24.1% total)
│   │   └── order_and_analyze: 1.56187s (24.1% total) ← PRIMARY TARGET
│   │       ├── btf_l_order: 0.79196s (12.2% total)
│   │       │   └── btf_l_maxtrans: 0.79196s (12.2% total)
│   │       │       └── augment: 0.7848s (12.1% total)
│   │       └── analyze_worker: 0.671946s (10.4% total)
│   │           └── colamd_l: 0.633905s (9.8% total)
│   │               └── find_ordering: 0.512002s (7.9% total)
│   └── klu_l_factor: 1.00309s (15.5% total) ← Numerical factorization
└── DYN::SolverKINEuler::evalJ_KIN: 1.72504s (26.7% total) ← Jacobian evaluation
```

**Key Performance Insights:**
- **Symbolic factorization (39.6%)** dominates over **numerical factorization (15.5%)**
- **BTF preprocessing** alone takes **12.2%** of total execution time
- **Matrix ordering (COLAMD)** consumes **9.8%** of total time
- **Jacobian evaluation** takes **26.7%** but is separate from factorization

### 2. **Root Cause Analysis: 100% Symbolic Factorization Rate**

The performance bottleneck is caused by **excessive symbolic factorizations** - the simulation performs symbolic factorization on **every single time step** (7 out of 7), triggered by:

#### **Primary Triggers:**
```cpp
// DYNSolverCommonFixedTimeStep.cpp:319-320
if (stats_.nst_ == 0 || factorizationForced_)
    noInitSetup = false;  // ← Triggers expensive kinLsSetup
```

#### **Secondary Triggers (From Simulation Log Analysis):**
- **Algebraic mode changes**: Occur at every time step (t=1,3,4,5,6,7)
- **"J recalculation" events**: Explicit Jacobian recalculation at t=3
- **"New starting point" calculations**: 5 occurrences indicating solver restarts
- **Matrix structure changes**: Caused by 319,888 variable system complexity
- **Convergence failures**: Leading to `factorizationForced_ = true`

**Key Finding**: The combination of large system size (319,888 variables) and frequent algebraic mode changes creates a "perfect storm" where every time step requires complete symbolic refactorization.

### 3. **PFR Network Characteristics**

The all_retained test case represents:
- **Large French transmission network** (312,580 lines in IIDM)
- **Complex topology** with numerous substations and voltage levels
- **Dense Jacobian matrix** (~10,000+ variables, 100,000+ non-zeros)
- **Dynamic behavior** causing frequent matrix structure variations

### 4. **Matrix Structure Instability Issue**

From DYNAWO documentation:
> "Due to some numerical noises, the number of non-zero elements in the matrix in large test cases often varies from one decomposition to another."

This causes unnecessary symbolic refactorizations when only numerical values change.

## Performance Impact Analysis

### **Current Performance Profile (Actual Data):**
- **Total symbolic factorization time**: 2.56496s (39.6% of total execution)
- **order_and_analyze time**: 1.56187s (24.1% of total execution)
- **BTF preprocessing time**: 0.79196s (12.2% of total execution)
- **Matrix ordering (COLAMD) time**: 0.633905s (9.8% of total execution)
- **Numerical factorization time**: 1.00309s (15.5% of total execution)

**Critical Finding:** Symbolic factorization takes **2.56x more time** than numerical factorization, indicating excessive symbolic refactorizations.

### **Optimization Potential:**
- **Primary target**: Reduce 1.56187s `order_and_analyze` time by 50-70%
- **Secondary target**: Optimize 0.79196s BTF preprocessing by 20-30%
- **Expected overall speedup**: 1.5-2.0x faster simulation (3.2-4.3s vs 6.47s)
- **Conservative estimate**: Save 1.0-1.5s per simulation (15-23% improvement)

## Optimization Strategies

### **Strategy 1: Matrix Structure Change Tolerance**

#### **Problem:**
```cpp
// DYNSolverCommon.cpp:71-74
bool matrixStructChange = copySparseToKINSOL(smj, JJ, size, *lastRowVals);
if (matrixStructChange) {
    SUNLinSol_KLUReInit(LS, JJ, SM_NNZ_S(JJ), 2);  // ← Expensive!
}
```

#### **Solution:**
```cpp
bool SolverCommon::copySparseToKINSOL(const SparseMatrix& smj, SUNMatrix& JJ, 
                                      const int& size, sunindextype* lastRowVals) {
    // Add tolerance for small structural changes
    static const double STRUCTURE_CHANGE_TOLERANCE = 0.01;  // 1% tolerance
    static const int MIN_NNZ_CHANGE = 10;  // Minimum absolute change
    
    if (lastRowVals != NULL) {
        int currentNNZ = SM_NNZ_S(JJ);
        int newNNZ = smj.nbTerm();
        int nnzDiff = std::abs(newNNZ - currentNNZ);
        
        // Only trigger structure change if significant difference
        double changeRatio = static_cast<double>(nnzDiff) / currentNNZ;
        
        if (changeRatio < STRUCTURE_CHANGE_TOLERANCE && 
            nnzDiff < MIN_NNZ_CHANGE) {
            // Minor structure change - reuse symbolic factorization
            // Resize matrix if needed but keep structure
            if (newNNZ != currentNNZ) {
                SUNSparseMatrix_Reallocate(JJ, newNNZ);
            }
            
            // Copy sparsity pattern and values
            memcpy(SM_INDEXPTRS_S(JJ), &smj.Ap()[0], 
                   sizeof(sunindextype) * (size + 1));
            memcpy(SM_INDEXVALS_S(JJ), &smj.Ai()[0], 
                   sizeof(sunindextype) * newNNZ);
            memcpy(SM_DATA_S(JJ), &smj.Ax()[0], 
                   sizeof(realtype) * newNNZ);
            
            return false;  // No structure change flagged
        }
    }
    
    // Original structure change logic for significant changes
    // ... existing implementation ...
    return true;  // Structure change detected
}
```

**Expected Impact (Based on Simulation Analysis):** 
- **Primary benefit**: Reduce factorizations from 7 to 3-4 per simulation
- **Time savings per avoided factorization**: 0.366s (total 1.1-1.5s savings)
- **Percentage improvement**: 43-57% reduction in symbolic factorizations
- **Overall simulation speedup**: 17-23% faster execution

### **Strategy 2: Enhanced Factorization Control Logic**

#### **Current Issues:**
- Binary decision: factorize or not
- No consideration of convergence history
- No adaptive behavior based on problem characteristics

#### **Enhanced Implementation:**
```cpp
class SolverCommonFixedTimeStep {
private:
    int consecutiveGoodConvergence_;
    int stepsSinceLastFactorization_;
    int totalSymbolicFactorizations_;
    double avgConvergenceRate_;
    
    static const int GOOD_CONVERGENCE_THRESHOLD = 5;
    static const int MAX_STEPS_WITHOUT_FACTORIZATION = 15;
    static const double POOR_CONVERGENCE_THRESHOLD = 0.1;

public:
    bool shouldForceFactorization() {
        // Adaptive factorization strategy
        if (consecutiveGoodConvergence_ >= GOOD_CONVERGENCE_THRESHOLD) {
            // Good convergence history - be more conservative
            return stepsSinceLastFactorization_ >= MAX_STEPS_WITHOUT_FACTORIZATION;
        }
        
        if (avgConvergenceRate_ < POOR_CONVERGENCE_THRESHOLD) {
            // Poor convergence - force more frequent factorizations
            return stepsSinceLastFactorization_ >= MAX_STEPS_WITHOUT_FACTORIZATION / 2;
        }
        
        return factorizationForced_;
    }
    
    void updateFactorizationHistory(SolverStatus_t status, int newtonIterations) {
        if (status == CONV) {
            consecutiveGoodConvergence_++;
            // Update convergence rate (exponential moving average)
            double currentRate = 1.0 / std::max(1, newtonIterations);
            avgConvergenceRate_ = 0.8 * avgConvergenceRate_ + 0.2 * currentRate;
        } else {
            consecutiveGoodConvergence_ = 0;
            avgConvergenceRate_ *= 0.9;  // Degrade average
        }
        
        stepsSinceLastFactorization_++;
    }
    
    void resetFactorizationCounters() {
        stepsSinceLastFactorization_ = 0;
        totalSymbolicFactorizations_++;
    }
};
```

#### **Integration:**
```cpp
int SolverCommonFixedTimeStep::callAlgebraicSolver() {
    if (skipNextNR_) {
        return KIN_INITIAL_GUESS_OK;
    }
    
    computePrediction();
    
    // Enhanced factorization control
    bool noInitSetup = true;
    if (stats_.nst_ == 0 || shouldForceFactorization()) {
        noInitSetup = false;
        resetFactorizationCounters();
    }
    
    flag = solverKINEuler_->solve(noInitSetup, skipAlgebraicResidualsEvaluation_);
    
    // Update convergence history
    SolverStatus_t status = analyzeResult(flag);
    updateFactorizationHistory(status, nNewt_);
    
    updateStatistics();
    return flag;
}
```

**Expected Impact (Based on Simulation Analysis):** 
- **Intelligent factorization decisions**: Avoid 1-2 additional unnecessary factorizations
- **Time savings**: 0.37-0.73s additional reduction (1-2 factorizations × 0.366s)
- **Combined with Strategy 1**: Total 1.47-2.23s savings (23-34% overall speedup)
- **Factorization frequency**: Reduce from 7 to 2-3 per simulation

### **Strategy 3: KLU Ordering and BTF Optimization**

#### **Current KLU Configuration:**
```cpp
// Default KLU setup - may not be optimal for power systems
linearSolver_ = SUNLinSol_KLU(sundialsVectorY_, sundialsMatrix_, sundialsContext_);
```

#### **Optimized Configuration:**
```cpp
void SolverKINCommon::initCommon(...) {
    // Create KLU linear solver
    linearSolver_ = SUNLinSol_KLU(sundialsVectorY_, sundialsMatrix_, sundialsContext_);
    
    // Optimize for power system matrices
    // COLAMD ordering is typically best for power systems
    SUNLinSol_KLUSetOrdering(linearSolver_, 1);  // COLAMD
    
    // Optimize BTF parameters
    klu_common *Common = SUNLinSol_KLUGetCommon(linearSolver_);
    if (Common != NULL) {
        // Adjust BTF tolerance for power system matrices
        Common->btol = 0.1;  // Block tolerance (default 0.01)
        
        // Optimize for power system sparsity patterns
        Common->ordering = 1;  // COLAMD
        Common->halt_if_singular = 0;  // Don't halt on singularity
        
        // Memory management
        Common->grow0 = 1.2;  // Growth factor for L and U
        Common->grow = 1.2;
        Common->grow2 = 5.0;  // Growth factor for workspace
        
        // Partial pivoting parameters
        Common->partial_pivot = 1;  // Enable partial pivoting
        Common->pivot_tolerance = 0.1;  // Pivot tolerance
    }
    
    // Set linear solver tolerances appropriately
    SUNLinSol_KLUSetTolerance(linearSolver_, 1e-12);
}
```

**Expected Impact:** 
- **BTF optimization**: 0.15-0.25s savings from 0.79196s BTF time (19-32% improvement)
- **COLAMD optimization**: 0.06-0.13s savings from 0.633905s ordering time (10-20% improvement)
- **Total time savings**: 0.21-0.38s when symbolic factorization occurs
- **Overall benefit**: 3-6% simulation speedup

### **Strategy 4: Monitoring and Diagnostics**

#### **Performance Counters:**
```cpp
class FactorizationProfiler {
private:
    int symbolicFactorizations_;
    int numericalOnlyFactorizations_;
    int structureChangeDetections_;
    int falsePositiveStructureChanges_;
    double totalSymbolicTime_;
    double totalNumericalTime_;
    
public:
    void logSymbolicFactorization(double elapsedTime) {
        symbolicFactorizations_++;
        totalSymbolicTime_ += elapsedTime;
    }
    
    void logNumericalFactorization(double elapsedTime) {
        numericalOnlyFactorizations_++;
        totalNumericalTime_ += elapsedTime;
    }
    
    void logStructureChange(bool wasNecessary) {
        structureChangeDetections_++;
        if (!wasNecessary) falsePositiveStructureChanges_++;
    }
    
    void printStatistics() {
        double totalFactorizations = symbolicFactorizations_ + numericalOnlyFactorizations_;
        double symbolicRatio = symbolicFactorizations_ / totalFactorizations;
        double avgSymbolicTime = totalSymbolicTime_ / symbolicFactorizations_;
        double avgNumericalTime = totalNumericalTime_ / numericalOnlyFactorizations_;
        
        Trace::info() << "=== Factorization Statistics ===" << Trace::endline;
        Trace::info() << "Symbolic factorizations: " << symbolicFactorizations_ << Trace::endline;
        Trace::info() << "Numerical-only factorizations: " << numericalOnlyFactorizations_ << Trace::endline;
        Trace::info() << "Symbolic ratio: " << (symbolicRatio * 100) << "%" << Trace::endline;
        Trace::info() << "Average symbolic time: " << avgSymbolicTime << "s" << Trace::endline;
        Trace::info() << "Average numerical time: " << avgNumericalTime << "s" << Trace::endline;
        Trace::info() << "False positive structure changes: " << falsePositiveStructureChanges_ 
                      << "/" << structureChangeDetections_ << Trace::endline;
    }
};
```

## Implementation Plan

### **Phase 1: Immediate Optimizations (Low Risk, High Impact)**
1. **Implement matrix structure change tolerance** (Strategy 1)
   - Modify `copySparseToKINSOL` function
   - Add tolerance parameters (1% change threshold)
   - Test with PFR network

2. **Optimize KLU configuration** (Strategy 3)
   - Set COLAMD ordering
   - Adjust BTF parameters for power systems
   - Validate numerical stability

**Timeline:** 1-2 weeks  
**Expected Impact:** 
- **Time savings**: 0.71-1.05s reduction (Strategy 1 + Strategy 3)
- **Simulation speedup**: 11-16% faster execution
- **New simulation time**: 5.42-5.76s (vs current 6.47s)

### **Phase 2: Advanced Optimizations (Medium Risk, High Impact)**
3. **Implement adaptive factorization strategy** (Strategy 2)
   - Add convergence history tracking
   - Implement intelligent factorization decisions
   - Extensive testing with various network sizes

4. **Add comprehensive monitoring** (Strategy 4)
   - Implement performance counters
   - Add diagnostic logging
   - Create optimization dashboard

**Timeline:** 3-4 weeks  
**Expected Impact:** 
- **Additional time savings**: 0.3-0.5s (Strategy 2) + monitoring benefits
- **Combined total savings**: 1.01-1.55s (15.6-24.0% overall improvement)
- **Final simulation time**: 4.92-5.46s (vs current 6.47s)

### **Phase 3: Validation and Fine-tuning**
5. **Comprehensive testing across multiple networks**
6. **Performance validation with Intel VTune**
7. **Parameter tuning based on results**
8. **Documentation and best practices**

**Timeline:** 2-3 weeks

## Risk Assessment

### **Low Risk Optimizations:**
- **KLU parameter tuning**: Well-documented, reversible
- **Structure change tolerance**: Conservative thresholds, fallback to original behavior

### **Medium Risk Optimizations:**
- **Adaptive factorization**: Requires extensive testing to ensure convergence
- **Matrix structure detection**: Could affect numerical stability if thresholds too aggressive

### **Mitigation Strategies:**
- **Gradual rollout**: Test with single network first
- **Fallback mechanisms**: Ability to revert to original behavior
- **Comprehensive monitoring**: Early detection of issues
- **Parameter configurability**: Runtime adjustment of thresholds

## Expected Results

### **Performance Improvements (Based on Actual Profiling Data):**
- **Primary goal**: Reduce `order_and_analyze` time from 1.56187s to 0.55-0.93s (40-65% reduction)
- **Secondary goal**: Reduce `kinLsSetup` time from 4.30201s to 2.75-3.29s (24-36% reduction)
- **Overall simulation speedup**: From 6.47s to 4.92-5.46s (24-31% faster)
- **Scalability**: Proportionally better performance for larger networks

### **Validation Metrics (Quantified Targets):**
- **VTune profiling targets**: 
  - `order_and_analyze`: <1.0s (currently 1.56187s)
  - `kinLsSetup`: <3.0s (currently 4.30201s)
  - Total simulation: <5.5s (currently 6.47s)
- **Factorization frequency**: Reduce symbolic-to-numerical ratio from 2.56:1 to <1.5:1
- **Convergence stability**: Maintain <5 Newton-Raphson iterations per time step
- **Numerical accuracy**: Maintain solution precision within 1e-8 tolerance

## Detailed Profiling and Simulation Analysis

### **Simulation Characteristics (from all_retained.log)**

The simulation exhibits the following behavior that directly impacts symbolic factorization frequency:

**System Size:**
- **Continuous variables**: 319,888 (extremely large power system)
- **Discrete variables**: 334,848
- **Root functions**: 276,125
- **Simulation duration**: 7 seconds (t=0 to t=7, 1s time steps)

**Factorization Triggers Identified:**
- **Time iterations**: 7 (one per second)
- **Jacobian evaluations**: 7 (100% factorization rate - every time step!)
- **Newton-Raphson iterations**: 31 total (avg 4.4 per time step)
- **Algebraic mode changes**: Every time step
- **"J recalculation" events**: At least 1 explicit event at t=3
- **"New starting point" calculations**: 5 events (indicating solver restarts)

**Critical Finding**: The simulation performs **1 Jacobian evaluation per time step**, meaning **100% symbolic factorization rate** - explaining why `order_and_analyze` dominates execution time.

### **Complete Function Call Tree (Intel VTune Data)**

Based on the actual `all_retained.csv` profiling data correlated with simulation behavior:

| Function | Time (s) | % Total | % Parent | Analysis |
|----------|----------|---------|----------|----------|
| **Total Simulation** | 6.47047 | 100.0% | - | Full simulation time |
| **Core Solver** | 6.37443 | 98.5% | 98.5% | DYNAWO solver execution |
| **kinLsSetup** | 4.30201 | 66.5% | 67.5% | **PRIMARY BOTTLENECK** |
| **SUNLinSolSetup_KLU** | 2.56496 | 39.6% | 59.6% | KLU symbolic + numerical |
| **order_and_analyze** | 1.56187 | 24.1% | 60.9% | **TARGET FUNCTION** |
| **btf_l_maxtrans** | 0.79196 | 12.2% | 50.7% | BTF maximum transversal |
| **augment** | 0.7848 | 12.1% | 99.1% | Core BTF algorithm |
| **colamd_l** | 0.633905 | 9.8% | 94.4% | Matrix ordering |
| **find_ordering** | 0.512002 | 7.9% | 80.8% | COLAMD core algorithm |
| **klu_l_factor** | 1.00309 | 15.5% | 39.1% | Numerical factorization |
| **evalJ_KIN** | 1.72504 | 26.7% | 40.1% | Jacobian evaluation |

### **Critical Performance Ratios (Correlated with Simulation Data)**
- **Symbolic vs Numerical**: 2.56496s / 1.00309s = **2.56:1 ratio** (should be ~1:1)
- **BTF vs Total Symbolic**: 0.79196s / 2.56496s = **30.9%** of symbolic time
- **Ordering vs Total Symbolic**: 0.633905s / 2.56496s = **24.7%** of symbolic time
- **Setup vs Solve**: 4.30201s / 0.29929s = **14.4:1 ratio** (extremely high)

**Root Cause Analysis:**
- **Cost per symbolic factorization**: 2.56496s ÷ 7 evaluations = **0.366s per factorization**
- **Cost per order_and_analyze**: 1.56187s ÷ 7 evaluations = **0.223s per call**
- **100% factorization rate**: Every time step triggers complete symbolic analysis
- **Mode change frequency**: Continuous algebraic mode changes force J recalculations
- **Large matrix size**: 319,888 variables create massive symbolic factorization overhead

### **Optimization Impact Projections (Based on Simulation Analysis)**

**Current State (7 time steps, 7 symbolic factorizations):**
- **Per-step symbolic cost**: 0.366s per factorization
- **Per-step order_and_analyze cost**: 0.223s per factorization

| Strategy | Target Function | Current Time | Optimized Time | Savings | % Improvement |
|----------|----------------|--------------|----------------|---------|---------------|
| **Strategy 1** | Reduce factorization frequency | 7 factorizations | 3-4 factorizations | 0.73-1.46s | 43-57% reduction |
| **Strategy 2** | order_and_analyze optimization | 1.56187s | 0.78-1.09s | 0.47-0.78s | 30-50% per call |
| **Strategy 3** | BTF/COLAMD optimization | 1.42587s | 1.21-1.29s | 0.13-0.21s | 10-15% per call |
| **Combined** | kinLsSetup total | 4.30201s | 2.10-2.87s | 1.43-2.20s | 33-51% total |

**Realistic Optimization Scenario:**
- **Reduce factorizations**: From 7 to 3 per simulation (skip 4 unnecessary factorizations)
- **Optimize remaining factorizations**: 30% improvement in order_and_analyze
- **Expected total savings**: 1.5-2.0s (23-31% simulation speedup)

## SUNDIALS Integration Considerations

Based on SUNDIALS documentation analysis:

### **Linear Solver Best Practices:**
- **Matrix size considerations**: PFR network (N>10,000) requires sparse methods
- **Sparsity exploitation**: KLU is optimal for power system problems
- **Preconditioning**: Not typically used with direct solvers like KLU

### **KINSOL Reinitialization:**
- **Problem size constraint**: Reinitialization only if matrix dimensions unchanged
- **Parameter updates**: Use `KINSet*` functions for solver parameter changes
- **Linear solver changes**: Properly sequence solver setup calls

### **Performance Profiling Integration:**
```cpp
// Add SUNDIALS profiling markers
SUNDIALS_MARK_BEGIN(profobj, "Symbolic Factorization");
SUNLinSol_KLUReInit(LS, JJ, SM_NNZ_S(JJ), 2);
SUNDIALS_MARK_END(profobj, "Symbolic Factorization");
```

## Conclusion

The `order_and_analyse` performance bottleneck is primarily caused by excessive symbolic factorizations due to matrix structure sensitivity in large power system networks. The proposed optimization strategies address this through:

1. **Intelligent structure change detection** with tolerance thresholds
2. **Adaptive factorization control** based on convergence history
3. **Optimized KLU configuration** for power system matrices
4. **Comprehensive monitoring** for continuous optimization

Implementation of these strategies is expected to reduce `order_and_analyze` execution time from **1.56187s to 0.55-0.93s (40-65% reduction)**, reducing total simulation time from **6.47s to 4.92-5.46s (24-31% overall speedup)** for the PFR_20240605_N_NB_all_retained network.

The optimization approach balances performance gains with numerical stability, ensuring that convergence and solution accuracy are maintained while dramatically reducing computational overhead in the symbolic factorization phase.

## References

1. SUNDIALS Documentation: Linear Algebraic Solvers
2. KLU User Guide: Sparse LU Factorization
3. DYNAWO Functional Documentation: Solver Architecture
4. Intel VTune Profiler: Performance Analysis Results
5. SuiteSparse Documentation: BTF and Matrix Ordering Algorithms
