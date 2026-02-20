# Dynawo Solver Performance Optimization Strategy

**Author:** Sustainable Power Systems Lab (SPS-L)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** December 2025

---

## Executive Summary

This document presents a performance optimization strategy for the Dynawo power system simulator, targeting a **30-45% overall speedup** in time-domain simulations. The strategy is based on Intel VTune profiling of representative large-scale French transmission network simulations (~300k+ variables).

**Key Finding:** Beyond known bottlenecks, profiling revealed a **hidden data structure hotspot** consuming 10-12% of execution time that was not identified in previous analyses.

---

## Performance Bottleneck Analysis

### Test Case Characteristics
- **Network**: French transmission network (PFR_20240605)
- **System Size**: ~300,000+ variables
- **Simulation Duration**: 100 seconds with frequent mode changes
- **Events**: Generator trips, tap changes, SVC operations, VCS arming/disarming

### Where Time Is Spent

| Component | % of Total Time | Notes |
|-----------|-----------------|-------|
| **KLU Symbolic Factorization** | 30-32% | Primary bottleneck |
| ├─ BTF preprocessing | 15-17% | Graph algorithms |
| └─ Matrix ordering (COLAMD) | 12-14% | Fill-reducing ordering |
| **Jacobian Evaluation** | 27-30% | Parallelizable |
| ├─ ModelNetwork::evalJt | 18-21% | Network model |
| └─ Adept AD | 7-9% | Automatic differentiation |
| **KLU Numerical Factorization** | 17-18% | Already efficient |
| **Derivatives Data Structure** | **10-12%** | **Hidden hotspot** |
| ├─ Map insertions | 3-5% | Memory allocation |
| ├─ Tree iteration | 4-5% | Cache-unfriendly |
| └─ Map clearing | 2-3% | O(n) destruction |
| **Residual Evaluation** | 6-8% | Minor target |

---

## Optimization Summary Table

| # | Optimization | Expected Speedup | Effort | Risk | Status |
|---|-------------|------------------|--------|------|--------|
| **1** | Replace `std::map` in Derivatives class | 8-10% | Low | Low | 🔄 Pending |
| **2** | Adaptive Factorization Control | 5-8% | Low | Low | ✅ Implemented |
| **3** | KLU Numerical-only Refactorization | 5-7% | Medium | Low | 🔄 Pending |
| **4** | Matrix Structure Change Tolerance | 3-5% | Low | Low | ✅ Implemented |
| **5** | KLU COLAMD Ordering | 2-4% | Low | Low | ✅ Implemented |
| **6** | Tune Factorization Thresholds | 2-3% | Low | Low | 🔄 Tuning needed |
| **7** | OpenMP Component Parallelization | 8-12% | Medium | Medium | 📋 Designed |
| **8** | OpenMP SubModel Parallelization | 3-5% | Medium | Medium | 📋 Designed |
| **9** | Partial Jacobian Updates | 10-20% | High | High | 📋 Designed |

**Legend:** ✅ Implemented | 🔄 Pending | 📋 Design only

---

## Detailed Optimization Descriptions

### 1. Replace `std::map` with Flat Vector in Derivatives Class ⭐⭐⭐

**Priority:** CRITICAL  
**Expected Speedup:** 8-10%  
**Implementation Effort:** Low (2-3 days)  
**Risk:** Low

**Problem:**  
The `Derivatives` class uses `std::map<int, double>` for storing Jacobian derivatives. This causes:
- O(log n) insertion time per value
- O(n) clearing time (tree destruction)
- Poor cache locality (scattered memory allocations)
- Frequent `operator new` calls visible in profiling

**Solution:**  
Replace with a flat vector structure:
- Pre-allocated `std::vector<double>` for direct O(1) indexing
- Separate `std::vector<int>` tracking non-zero indices
- O(nnz) reset instead of O(n) clearing

**Files to Modify:**
- `DYNDerivative.h`
- `DYNDerivative.cpp`
- `DYNModelBus.cpp`

---

### 2. Adaptive Factorization Control ⭐⭐⭐

**Priority:** High  
**Expected Speedup:** 5-8%  
**Implementation Effort:** Low  
**Risk:** Low  
**Status:** ✅ Implemented, ⏳ Testing pending

**Problem:**  
Symbolic factorization (30% of time) is triggered too frequently, even when the Jacobian structure hasn't changed significantly.

**Solution:**  
Intelligent factorization scheduling based on convergence history:
- Track consecutive successful convergences
- Track steps since last factorization
- Monitor average convergence rate
- Only force factorization when numerically necessary

**Current Thresholds:**
```
Good convergence threshold: 5 consecutive successes
Max steps without factorization: 15
Poor convergence threshold: 0.1 (convergence rate)
```

---

### 3. KLU Numerical-only Refactorization ⭐⭐

**Priority:** High  
**Expected Speedup:** 5-7%  
**Implementation Effort:** Medium (3-5 days)  
**Risk:** Low

**Problem:**  
When matrix values change but structure remains the same, full symbolic+numerical factorization is performed.

**Solution:**  
Use SUNDIALS API to perform numerical-only refactorization:
```cpp
if (structureChanged) {
    SUNLinSol_KLUReInit(LS, JJ, nnz, 2);  // Full refactorization
} else {
    SUNLinSol_KLUReInit(LS, JJ, nnz, 1);  // Numerical only
}
```

This skips expensive BTF preprocessing and matrix ordering when only numerical values change.

---

### 4. Matrix Structure Change Tolerance ⭐⭐

**Priority:** Medium  
**Expected Speedup:** 3-5%  
**Implementation Effort:** Low  
**Risk:** Low  
**Status:** ✅ Implemented, ⏳ Testing pending

**Problem:**  
Minor numerical noise in matrix structure detection triggers unnecessary symbolic refactorizations.

**Solution:**  
Apply tolerance thresholds before flagging structure change:
- Only trigger if NNZ change ≥ 1% (relative) OR ≥ 10 (absolute)
- Then verify actual structure via memory comparison

---

### 5. KLU COLAMD Ordering ⭐

**Priority:** Medium  
**Expected Speedup:** 2-4%  
**Implementation Effort:** Low  
**Risk:** Low  
**Status:** ✅ Implemented, ⏳ Testing pending

**Problem:**  
Default AMD ordering may not be optimal for power system matrices.

**Solution:**  
Explicitly set COLAMD ordering which typically performs better for power system Jacobians:
```cpp
SUNLinSol_KLUSetOrdering(linearSolver_, 1);  // 1 = COLAMD
```

**Note:** Advanced KLU parameters (btol, grow factors, pivot tolerance) are not accessible through the SUNDIALS API.

---

### 6. Tune Factorization Thresholds ⭐

**Priority:** Medium  
**Expected Speedup:** 2-3% (additional)  
**Implementation Effort:** Low (1 day)  
**Risk:** Low

**Current vs Proposed Thresholds:**

| Parameter | Current | Proposed |
|-----------|---------|----------|
| Good convergence threshold | 5 | 3 |
| Max steps without factorization | 15 | 25 |
| Poor convergence threshold | 0.1 | 0.05 |

**Additional Logic:**
- Skip factorization if Newton converged in ≤2 iterations AND good convergence streak

---

### 7. OpenMP Component Parallelization ⭐⭐

**Priority:** Medium-High  
**Expected Speedup:** 8-12% (on 4+ cores)  
**Implementation Effort:** Medium (1-2 weeks)  
**Risk:** Medium

**Concept:**  
Parallelize component derivative evaluation in `ModelNetwork::evalJt()`:

```cpp
const auto& components = getComponents();
#pragma omp parallel for schedule(dynamic) if(components.size() > 100)
for (size_t i = 0; i < components.size(); ++i) {
    components[i]->evalDerivatives(t);
}
```

**Requirements:**
- Thread-safe derivative accumulation (requires Optimization #1 first)
- Thread-local storage or atomic operations
- Careful handling of shared bus derivatives

---

### 8. OpenMP SubModel Parallelization ⭐

**Priority:** Medium  
**Expected Speedup:** 3-5%  
**Implementation Effort:** Medium (1 week)  
**Risk:** Medium

**Concept:**  
Parallelize submodel Jacobian evaluation in `ModelMulti::evalJt()`:

```cpp
#pragma omp parallel for schedule(dynamic) if(subModels_.size() > 10)
for (size_t i = 0; i < subModels_.size(); ++i) {
    subModels_[i]->evalJtSub(t, cj, localOffset, jt);
}
```

**Requirements:**
- Pre-computed submodel offsets
- Non-overlapping Jacobian regions per submodel

---

### 9. Partial Jacobian Updates ⭐

**Priority:** Low (future work)  
**Expected Speedup:** 10-20% (mode-change scenarios)  
**Implementation Effort:** High (2-4 weeks)  
**Risk:** High

**Concept:**  
Only recompute Jacobian entries for components affected by mode changes, not the entire matrix.

**Deferred due to:**
- Complex dependency tracking
- Risk of numerical errors
- High implementation effort

---

## Implementation Roadmap

### Phase 0: Foundation (Current State)

| Optimization | Status | Expected Impact |
|-------------|--------|-----------------|
| Matrix structure change tolerance | ✅ Code complete | 3-5% |
| KLU COLAMD ordering | ✅ Code complete | 2-4% |
| Adaptive factorization control | ✅ Code complete | 5-8% |
| **Phase 0 Total** | **Needs validation** | **10-17%** |

### Phase 1: Quick Wins (Recommended Next)

| Optimization | Effort | Expected Impact |
|-------------|--------|-----------------|
| #1 Derivatives `std::map` replacement | 3 days | 8-10% |
| #6 Threshold tuning | 1 day | 2-3% |
| #3 KLU numerical-only refactorization | 4 days | 5-7% |
| **Phase 1 Total** | **~8 days** | **15-20%** |

### Phase 2: Parallelization (If Needed)

| Optimization | Effort | Expected Impact |
|-------------|--------|-----------------|
| #7 OpenMP component parallelization | 1-2 weeks | 8-12% |
| #8 OpenMP submodel parallelization | 1 week | 3-5% |
| **Phase 2 Total** | **~3 weeks** | **11-17%** |

### Cumulative Expected Speedup

| Scenario | Expected Speedup |
|----------|------------------|
| Phase 0 only | 10-17% |
| Phase 0 + Phase 1 | **25-37%** |
| Phase 0 + Phase 1 + Phase 2 | **36-54%** |

---

## Risk Assessment

| Optimization | Risk Level | Mitigation Strategy |
|-------------|------------|---------------------|
| Derivatives vector replacement | **Low** | Extensive unit testing; same numerical results |
| Adaptive factorization | **Low** | Fallback to forced factorization on convergence issues |
| KLU numerical refactorization | **Low** | Only applied when structure unchanged |
| Matrix structure tolerance | **Low** | Conservative thresholds (1%) |
| KLU COLAMD ordering | **Low** | Standard SUNDIALS API |
| Threshold tuning | **Low** | Can easily revert to original values |
| OpenMP parallelization | **Medium** | ThreadSanitizer validation; careful synchronization |
| Partial Jacobian updates | **High** | Deferred to Phase 3 |

---

## Validation Strategy

### Validation Approach

1. **Numerical Accuracy:** Compare state variables with ≤1e-4 tolerance
2. **Event Sequence:** Verify identical timeline.xml
3. **Performance:** Measure with Intel VTune profiling

### Test Networks

| Network | Purpose | Variables |
|---------|---------|-----------|
| IEEE14 | Correctness validation | ~100 |
| IEEE57 | Performance baseline | ~500 |
| Nordic | Realistic testing | ~5,000 |
| IEEE118 | Stress testing | ~2,000 |
| PFR France | Production validation | ~300,000 |

### Validation Commands

```bash
# Run baseline simulation
./dynawo simulate reference_case.jobs --output baseline/

# Run optimized simulation
./dynawo simulate reference_case.jobs --output optimized/

# Compare numerical results
python compare_results.py baseline/ optimized/ --tolerance 1e-4

# Compare event sequences
diff baseline/outputs/timeline.xml optimized/outputs/timeline.xml
```

---

## Configuration Parameters

### Solver Parameters (solver.par)

```xml
<!-- Structure Change Detection -->
<par type="DOUBLE" name="structureChangeTolerance" value="0.01"/>
<par type="INT" name="minNNZChange" value="10"/>

<!-- Adaptive Factorization -->
<par type="INT" name="goodConvergenceThreshold" value="5"/>
<par type="INT" name="maxStepsWithoutFactorization" value="15"/>
<par type="BOOL" name="enableAdaptiveFactorization" value="true"/>
```

### OpenMP Environment (Phase 2)

```bash
export OMP_NUM_THREADS=4
export OMP_SCHEDULE="dynamic,1"
export OMP_PROC_BIND=close
export OMP_PLACES=cores
```

---

## Recommendations

### Immediate Actions

1. **Validate Phase 0:** Build and test current implementations against reference simulations
2. **Measure baseline:** Profile current performance with Intel VTune
3. **Implement #1:** Derivatives data structure replacement (highest impact-to-effort ratio)

### Decision Points

| If Phase 0 achieves... | Then... |
|------------------------|---------|
| < 5% speedup | Review implementation, check if optimizations are active |
| 5-15% speedup | Proceed with Phase 1 |
| > 15% speedup | Phase 1 may reach 30% target alone |

| If Phase 0+1 achieves... | Then... |
|--------------------------|---------|
| < 25% speedup | Phase 2 parallelization recommended |
| 25-35% speedup | Phase 2 optional based on requirements |
| > 35% speedup | Target achieved, Phase 2 for stretch goals |

---

## Summary

| Metric | Value |
|--------|-------|
| **Target Speedup** | 30-45% |
| **Phase 0 (implemented)** | 10-17% expected |
| **Phase 1 (next)** | +15-20% expected |
| **Total Implementation Time** | 2-3 weeks |
| **Risk Level** | Low to Medium |

**Key Insight:** The combination of the Derivatives data structure optimization (#1) and the already-implemented adaptive factorization (#2) should deliver the majority of the target speedup with minimal risk.

---

**Contact:** info@sps-lab.org  
**Website:** https://sps-lab.org
