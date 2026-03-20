# Dynawo Optimization Roadmap

This document presents a detailed optimization plan for the Dynawo power system simulation tool. It covers 9 programming optimizations and 7 algorithmic optimizations, each with descriptions, expected speedup ranges, implementation effort and risk assessments, and implementation sketches. A phased roadmap at the end provides a structured plan for executing these improvements.

> **Developer feedback incorporated.** Sections marked "Developer Feedback (TRAISIM Discussion)" contain insights from Gautier Bureau, former lead Dynawo developer at RTE, gathered during a TRAISIM project technical meeting. His feedback has been used to validate suggestions, remove already-implemented items (P1 — now in upstream master), add a new high-value optimization (A7), and refine complexity/risk assessments.

All speedup estimates are relative to typical large-scale power system simulations (1000+ buses, 10-60 second simulation windows). Actual improvements will vary depending on network size, event density, and solver configuration.

---

## Reference Benchmark System

The primary benchmark for validating optimizations is the **Nordic test system** (`examples/DynaWaltz/Nordic/Nordic.jobs`):

- **Network:** 74 buses, 52 lines, 20 generators, 22 loads
- **Dynamic models:** 41 models (governors, AVRs, OELs, dynamic loads, etc.)
- **Simulation:** 0-175s DynaWaltz long-term stability, with NodeFault at bus 4032_401 and line disconnection
- **Solver:** SolverSIM (fixed-step) with algebraic restoration via KINSOL

For the SIM solver, the key hot path is:

```
solveStep → solveStepCommon → callAlgebraicSolver → SolverKINEuler::solve → KINSOL iterations
```

Each KINSOL iteration involves: `evalF_KIN` (residual), `evalJ_KIN` (Jacobian), and `solveCommon → KINSol` (linear solve). The `callAlgebraicSolver` and `setupNewAlgebraicRestoration` methods are instrumented with `PHASE_LINEAR_SOLVE` and `PHASE_REINIT` respectively.

An IDA solver comparison is also available via `performance-analysis/benchmarks/Nordic_IDA.jobs`, which runs the same Nordic system with `dynawo_SolverIDA` (variable-step). This allows direct comparison of IDA vs SIM solver performance characteristics.

---

## Table of Contents

1. [Programming Optimizations](#programming-optimizations)
   - [P1. OpenMP Jacobian Evaluation](#p1-openmp-jacobian-evaluation)
   - [P2. OpenMP SubModel Evaluation](#p2-openmp-submodel-evaluation)
   - [P3. Cache-Optimized Sparse Matrix Layout](#p3-cache-optimized-sparse-matrix-layout)
   - [P4. SIMD Vectorization in Residual Evaluation](#p4-simd-vectorization-in-residual-evaluation)
   - [P5. Memory Pool Allocator](#p5-memory-pool-allocator)
   - [P6. Reduce SUNDIALS N_Vector Copies](#p6-reduce-sundials-nvector-copies)
   - [P7. Profile-Guided Optimization (PGO)](#p7-profile-guided-optimization-pgo)
   - [P8. Link-Time Optimization (LTO)](#p8-link-time-optimization-lto)
   - [P9. GPU Acceleration for KLU](#p9-gpu-acceleration-for-klu)
2. [Algorithmic Optimizations](#algorithmic-optimizations)
   - [A1. Adaptive Factorization Control](#a1-adaptive-factorization-control)
   - [A2. Matrix Structure Change Tolerance](#a2-matrix-structure-change-tolerance)
   - [A3. KLU Numerical-Only Refactorization](#a3-klu-numerical-only-refactorization)
   - [A4. Improved COLAMD Ordering](#a4-improved-colamd-ordering)
   - [A5. Partial Jacobian Updates](#a5-partial-jacobian-updates)
   - [A6. Schur Complement Decomposition](#a6-schur-complement-decomposition)
   - [A7. Event Severity Classification for Reinit/Factorization Control](#a7-event-severity-classification-for-reinitfactorization-control)
3. [Phased Roadmap](#phased-roadmap)
   - [Phase 0: Quick Wins](#phase-0-quick-wins)
   - [Phase 1: Medium Effort](#phase-1-medium-effort)
   - [Phase 2: Advanced](#phase-2-advanced)
   - [Phase 3: Research / Exploratory](#phase-3-research--exploratory)
   - [Decision Points and Go/No-Go Criteria](#decision-points-and-gono-go-criteria)

---

## Programming Optimizations

### P1. OpenMP Jacobian Evaluation

**Expected Speedup:** 8-12%
**Implementation Effort:** Medium-High
**Risk Level:** Medium (practical integration risk is high until thread-safety audit and KLU lock-contention profiling pass — see Developer Feedback below)

#### Description

The Jacobian evaluation phase (`ModelMulti::evalJt()`) iterates over all submodels and evaluates each one's contribution to the global Jacobian matrix. In the current implementation, this loop is sequential: each submodel's Jacobian is computed one after another. Since submodels are largely independent during the evaluation phase (they read shared state but write to non-overlapping regions of the Jacobian matrix), this loop is amenable to OpenMP parallelization.

The main complication is that each submodel writes to a different row/column block of the sparse Jacobian. As long as the write regions do not overlap -- which is guaranteed by the block structure of the multi-model Jacobian -- the loop can be parallelized without locks. Care must be taken with thread-local scratch buffers and ensuring that no submodel modifies shared state during Jacobian evaluation.

The expected speedup depends on the number of submodels and their relative evaluation cost. For simulations with many generator models (each with its own Jacobian block), the speedup scales well with core count up to the number of submodels. For network-dominated simulations with few large submodels, the speedup is more modest.

#### Developer Feedback (TRAISIM Discussion)

Gautier Bureau (former Dynawo lead developer at RTE) provided critical context:

1. **KLU lock contention:** RTE previously attempted OpenMP parallelization for N-1 batch simulations (multiple independent simulations in parallel). They found that KLU has internal locks on shared data structures that caused contention and poor scaling. This affected inter-simulation parallelism but may also affect intra-simulation parallelism if KLU is called within the parallel region.

2. **Network model is a submodel:** The network model (`ModelNetwork`) is itself one of the submodels in the `ModelMulti` structure. Parallelizing the submodel loop and separately parallelizing within the network model would create nested OpenMP regions. Nested parallelism must be carefully managed (or avoided) to prevent thread oversubscription.

3. **RTE's historical priority:** RTE historically parallelized at the simulation level (N-1 scenarios) rather than within a single simulation, because their use case involves running many independent simulations. For real-time OTS (the TRAISIM use case), intra-simulation parallelism is the priority instead.

**Recommended approach:** Parallelize at the submodel `evalJt` level but treat `ModelNetwork` specially — either exclude it from the parallel loop (evaluate it sequentially before/after the other submodels) or ensure its internal parallelization is disabled when called from within a parallel region. Profile with `OMP_NESTED=false` first.

#### Implementation Sketch

```cpp
// In ModelMulti::evalJt()
void ModelMulti::evalJt(double t, double cj, SparseMatrix& Jt) {
  DYN_PROFILE_PHASE(PHASE_JACOBIAN_EVAL);

  const int nSubModels = static_cast<int>(subModels_.size());

  // Each submodel writes to its own block, no overlap
  #pragma omp parallel for schedule(dynamic, 1) if(nSubModels > 4)
  for (int i = 0; i < nSubModels; ++i) {
    // Thread-local scratch space for submodel Jacobian
    subModels_[i]->evalJt(t, cj, Jt, subModelOffsets_[i]);
  }

  // Sequential: assemble global matrix from submodel contributions
  // (only needed if submodels use thread-local buffers)
  assembleGlobalJacobian(Jt);
}
```

**CMake integration:**

```cmake
if(DYNAWO_OPENMP)
  find_package(OpenMP REQUIRED)
  target_link_libraries(dynawo_Solvers OpenMP::OpenMP_CXX)
  target_compile_definitions(dynawo_Solvers PRIVATE DYNAWO_OPENMP)
endif()
```

---

### P2. OpenMP SubModel Evaluation

**Expected Speedup:** 3-5%
**Implementation Effort:** Medium
**Risk Level:** Medium (practical integration risk is high until thread-safety audit confirms no shared mutable state in submodel evaluation paths)

#### Description

Beyond Jacobian evaluation, the residual evaluation (`evalF`) and root-finding (`evalG`) phases also iterate over submodels sequentially. Parallelizing these loops with OpenMP provides additional speedup, though the per-submodel computation is typically lighter than for Jacobian evaluation, so the benefit is smaller.

The residual evaluation computes F(t, y, y') for each submodel and writes to the corresponding portion of the global residual vector. Since each submodel owns a contiguous slice of the residual vector, parallelization is straightforward. The root evaluation similarly computes zero-crossing functions for each submodel independently.

The risk is that some submodels may have internal state that is not thread-safe, particularly those that cache intermediate computations. An audit of submodel implementations is required before enabling this optimization, and thread-safety annotations should be added to the `SubModel` interface.

#### Implementation Sketch

```cpp
// In ModelMulti::evalF()
void ModelMulti::evalF(double t, const double* y, const double* yp, double* f) {
  DYN_PROFILE_PHASE(PHASE_RESIDUAL_EVAL);

  const int n = static_cast<int>(subModels_.size());

  #pragma omp parallel for schedule(dynamic, 1) if(n > 4)
  for (int i = 0; i < n; ++i) {
    const int offset = subModelOffsets_[i];
    const int size = subModelSizes_[i];
    subModels_[i]->evalF(t,
                         y + offset,      // submodel's state slice
                         yp + offset,     // submodel's derivative slice
                         f + offset);     // submodel's residual slice
  }
}

// Similarly for evalG (root evaluation)
void ModelMulti::evalG(double t, const double* y, const double* yp, double* g) {
  DYN_PROFILE_PHASE(PHASE_ROOT_EVAL);

  const int n = static_cast<int>(subModels_.size());

  #pragma omp parallel for schedule(dynamic, 1) if(n > 4)
  for (int i = 0; i < n; ++i) {
    const int gOffset = subModelGOffsets_[i];
    subModels_[i]->evalG(t, y, yp, g + gOffset);
  }
}
```

---

### P3. Cache-Optimized Sparse Matrix Layout

**Expected Speedup:** 2-3%
**Implementation Effort:** High (staged rollout required — see delivery strategy below)
**Risk Level:** Medium-High (sparsity pattern stability must be validated across mode changes and discrete events before production use)

> **Code Analysis (March 2026).** Upstream code analysis reveals that the original framing of this item (row-major intermediates transposed before KLU) is **incorrect** — Dynawo already avoids explicit transposition through a deliberate CSR/transpose trick. However, the analysis uncovered real inefficiencies in the Jacobian assembly pipeline that this item should target instead. See the Code Analysis section below.
>
> **Important caveat:** Adept does not support sparse Jacobian computation — there is no `jacobian_with_pattern()` or coloring/compressed evaluation API. The dense `stack.jacobian()` call is unavoidable with the current AD library. The optimization therefore targets the extraction loop and allocation overhead, not the AD computation itself.

#### Description (Original — Superseded)

~~Sparse matrices in Dynawo are stored for use with the KLU direct solver from SuiteSparse. KLU operates on Compressed Sparse Column (CSC) format, which stores non-zero entries column by column. However, many operations during Jacobian assembly naturally produce data in row-major order, leading to an intermediate representation that must be transposed before KLU can use it.~~

#### Code Analysis: Actual Jacobian Assembly Pipeline

The `SparseMatrix` class (`DYNSparseMatrix.h`) uses `Ap_` (column pointers), `Ai_` (row indices), `Ax_` (values) — structurally a CSC format. Models fill it column-by-column via `changeCol()` + `addTerm(row, val)`.

**The CSR/transpose trick:** What models produce is `Jt` (the *transpose* of the Jacobian), not J itself. Both KINSOL (`SolverKINCommon.cpp:138`) and IDA (`DYNSolverIDA.cpp:257`) create the SUNMatrix with `CSR_MAT` instead of `CSC_MAT`:

```cpp
// Passing CSR_MAT indicates that we solve A'x = B
// - linear system using the matrix transpose -
// and not Ax = B (see sunlinsol_klu.c:149)
sundialsMatrix_ = SUNSparseMatrix(numF_, numF_, nnz, CSR_MAT, sundialsContext_);
```

When SUNDIALS/KLU sees `CSR_MAT`, it interprets the same `(Ap, Ai, Ax)` arrays as CSR rather than CSC — which is mathematically equivalent to transposing the matrix. So the net effect is: models produce Jt in CSC-structured arrays → KLU reads the same arrays as CSR → KLU effectively solves with J (the un-transposed Jacobian). **No explicit transposition step occurs.** The data flows as one contiguous memcpy from `SparseMatrix` to `SUNMatrix` via `copySparseToKINSOL()`.

**Two assembly paths exist:**

1. **Native C++ models** (e.g., `ModelBus`, `ModelLine`): Fill the `SparseMatrix` directly and sparsely — only non-zero terms are added. This is already efficient.

2. **Modelica-generated models** (via `ModelManager::evalJtAdept`): Adept AD computes a **dense** Jacobian (`stack.jacobian(&jac[0])` into a `vector<double>` of size `(2 × sizeY) × sizeY`), then iterates over all `sizeF × sizeY` entries to fill the sparse `Jt`, relying on `addTerm()`'s `doubleIsZero()` filter to skip zeros. This is an `O(n²)` dense→sparse conversion on every Jacobian evaluation.

**This path applies identically to both FixedTimeStep (SolverSIM) and IDA** — both use the same `SparseMatrix` → `copySparseToKINSOL()` → `SUNMatrix` pipeline.

#### Reframed Optimization Targets

Given the code analysis, this item should target three concrete inefficiencies:

1. **Compile-time structural index map for extraction loop (highest impact):** The `evalJtAdept` extraction loop (lines 356–368 in `DYNModelManager.cpp`) iterates all `sizeF × sizeY` entries and calls `addTerm()` for each, relying on `doubleIsZero()` to skip zeros. A pre-computed structural index map can skip known-zero positions entirely. **Note:** Adept does not support sparse Jacobian computation — `stack.jacobian()` will remain a dense O(n²) call. The optimization targets only the extraction/filling phase, not the AD computation itself. For Modelica submodels with high sparsity, this can still significantly reduce the number of `addTerm()` calls and improve cache locality.

2. **`copySparseToKINSOL` element-by-element copy:** Every Jacobian evaluation copies all `Ap`, `Ai`, `Ax` arrays from `SparseMatrix` → `SUNMatrix` via loops. If the `SparseMatrix` could directly use the SUNMatrix's memory (or a compatible layout), this copy could be eliminated.

3. **Repeated `SparseMatrix` allocation:** In both `evalJ_KIN` (SolverKINEuler) and `evalJ` (SolverIDA), a new `SparseMatrix smj` is created on the stack every call, with dynamic `std::vector` allocations (`Ai_`, `Ax_` grow in 1024-element blocks via `increaseReserve()`). Reusing a pre-allocated matrix would avoid repeated heap allocations.

#### Recommended Delivery Strategy (SolverSIM-first)

P3 should be delivered in three stages to manage the risk of incorrect sparsity assumptions:

**Phase A (prototype — Phase 1):** Build a compile-time structural index map from the first dense `stack.jacobian()` call. Cache the set of `(i, j)` pairs where `jac[i + j * sizeY] != 0`. On subsequent evaluations, `stack.jacobian()` still computes the full dense matrix, but the extraction loop iterates only over the cached non-zero positions instead of all `sizeF × sizeY` entries. Also pre-allocate the `SparseMatrix` (Target 3). This is a safe, incremental change.

**Phase B (conservative invalidation):** Invalidate the cached structural index map on any mode change or discrete variable change (`modeChangeType_t >= ALGEBRAIC_MODE`). Re-compute the index map from the next dense evaluation. Add instrumentation to count "unexpected nonzero outside structural map" events. This catches structural changes conservatively.

**Phase C (severe-only invalidation):** After stress-test validation confirms zero missed-structure events, restrict invalidation to severe events only (`modeChangeType_t >= ALGEBRAIC_J_UPDATE_MODE`). **This phase is gated by test evidence; the default must remain conservative (Phase B).**

> **Risk note:** Severe-only invalidation (Phase C) must never be promoted without passing the structural-sparsity validation checklist below. A missed structural nonzero means KLU solves with an incomplete Jacobian, which can cause silent convergence degradation or Newton failure.

#### Implementation Sketch (Reframed)

```cpp
// Phase A: Compile-time structural index map for extraction loop
// In ModelManager::evalJtAdept:

// stack.jacobian() remains unchanged — Adept has no sparse API
stack.jacobian(&jac[0]);  // dense (2*sizeY) * sizeY array

if (!structuralMapComputed_) {
  // One-time: scan dense Jacobian for non-zero structure
  structuralMap_.clear();
  for (unsigned int i = 0; i < sizeF(); ++i) {
    for (unsigned int j = 0; j < sizeY(); ++j) {
      const int indice = i + j * sizeY();
      const double term = coeff * jac[indice] + cj * jac[indice + offsetJPrim];
      if (!doubleIsZero(term))
        structuralMap_.push_back({i, j, indice});
    }
  }
  structuralMapComputed_ = true;
}

// Fast path: iterate only over known non-zero positions
for (unsigned int col = 0; col < sizeF(); ++col) {
  Jt.changeCol();
  // Binary search or pre-grouped iteration over entries for this column
}
for (const auto& entry : structuralMap_) {
  const double term = coeff * jac[entry.indice] + cj * jac[entry.indice + offsetJPrim];
  Jt.addTerm(entry.j + rowOffset, term);  // within correct column context
}

// Phase B: Invalidation on mode/discrete change
void ModelManager::notifyModeChange(modeChangeType_t type) {
  if (type >= ALGEBRAIC_MODE)
    structuralMapComputed_ = false;  // conservative: any mode change
}

// Target 3: Pre-allocated SparseMatrix (avoids per-call heap alloc)
class SolverKINEuler {
  SparseMatrix smj_;  // Reuse across evalJ_KIN calls
  void initJacobianMatrix(int size) {
    smj_.init(size, size);
    // Pre-reserve based on expected nnz from first evaluation
  }
};
```

---

### P4. SIMD Vectorization in Residual Evaluation

**Expected Speedup:** 3-5%
**Implementation Effort:** High
**Risk Level:** Medium

#### Description

The residual evaluation (`evalF`) and many submodel computations involve loops over arrays of doubles performing arithmetic operations (additions, multiplications, divisions). Modern CPUs have SIMD (Single Instruction, Multiple Data) units that can process 2-8 doubles simultaneously using AVX/AVX2/AVX-512 instructions, but the compiler often fails to auto-vectorize these loops due to pointer aliasing, complex control flow, or non-contiguous memory access.

By restructuring the innermost loops to use aligned, contiguous data and adding compiler hints (`__restrict__`, alignment attributes, `#pragma omp simd`), or by using explicit intrinsics for the most critical loops, we can significantly improve throughput. The target is the network model's residual computation, which evaluates admittance-based power flow equations across all branches -- a naturally vectorizable operation.

The risk is that explicit SIMD code is harder to maintain and may not be portable across CPU architectures. A pragmatic approach is to use `#pragma omp simd` and `__restrict__` qualifiers first, measure the improvement, and only resort to intrinsics for the hottest loops if auto-vectorization is insufficient.

#### Implementation Sketch

```cpp
// Before: scalar residual evaluation for network branches
void NetworkModel::evalBranchResiduals(
    const double* vRe, const double* vIm,
    const double* gBranch, const double* bBranch,
    double* fRe, double* fIm, int nBranches) {
  for (int k = 0; k < nBranches; ++k) {
    double dv_re = vRe[fromBus_[k]] - vRe[toBus_[k]];
    double dv_im = vIm[fromBus_[k]] - vIm[toBus_[k]];
    fRe[k] = gBranch[k] * dv_re - bBranch[k] * dv_im;
    fIm[k] = gBranch[k] * dv_im + bBranch[k] * dv_re;
  }
}

// After: SIMD-friendly with restrict and pragma hints
void NetworkModel::evalBranchResiduals(
    const double* __restrict__ vRe,
    const double* __restrict__ vIm,
    const double* __restrict__ gBranch,
    const double* __restrict__ bBranch,
    double* __restrict__ fRe,
    double* __restrict__ fIm,
    int nBranches) {

  // Gather voltage differences into contiguous buffers
  // (eliminates indirect indexing that blocks vectorization)
  alignas(64) double dvRe[nBranches];
  alignas(64) double dvIm[nBranches];

  for (int k = 0; k < nBranches; ++k) {
    dvRe[k] = vRe[fromBus_[k]] - vRe[toBus_[k]];
    dvIm[k] = vIm[fromBus_[k]] - vIm[toBus_[k]];
  }

  // This loop should now auto-vectorize
  #pragma omp simd aligned(dvRe, dvIm, gBranch, bBranch, fRe, fIm: 64)
  for (int k = 0; k < nBranches; ++k) {
    fRe[k] = gBranch[k] * dvRe[k] - bBranch[k] * dvIm[k];
    fIm[k] = gBranch[k] * dvIm[k] + bBranch[k] * dvRe[k];
  }
}
```

---

### P5. Memory Pool Allocator

**Expected Speedup:** 2-4%
**Implementation Effort:** Medium
**Risk Level:** Low

#### Description

During simulation, Dynawo frequently allocates and deallocates small objects: temporary vectors for intermediate computations, submodel scratch buffers, event structures, and string messages. Each allocation through the default `new`/`delete` involves a global heap lock (in multi-threaded scenarios) and general-purpose allocator overhead that is unnecessary for objects with predictable sizes and lifetimes.

A memory pool allocator pre-allocates blocks of memory for common object sizes and recycles them without returning to the system allocator. This eliminates allocation overhead, reduces fragmentation, and improves cache locality since frequently used objects are allocated from the same memory region. For single-threaded simulations, even a simple free-list allocator provides measurable improvement; for OpenMP-parallel simulations, thread-local pools eliminate contention on the global heap.

The implementation should be non-intrusive: a custom allocator class that can be used with `std::vector` and other STL containers via the allocator template parameter, or a global override for specific allocation sizes used in hot paths.

#### Implementation Sketch

```cpp
// Simple block pool allocator for fixed-size objects
template <typename T, size_t BlockSize = 4096>
class PoolAllocator {
  struct Block {
    alignas(T) char data[sizeof(T) * BlockSize];
    Block* next;
  };

  struct FreeNode {
    FreeNode* next;
  };

  Block* blockList_;
  FreeNode* freeList_;
  size_t currentIdx_;

public:
  PoolAllocator() : blockList_(nullptr), freeList_(nullptr), currentIdx_(BlockSize) {}

  ~PoolAllocator() {
    Block* b = blockList_;
    while (b) {
      Block* next = b->next;
      ::operator delete(b);
      b = next;
    }
  }

  T* allocate() {
    // First try the free list
    if (freeList_) {
      FreeNode* node = freeList_;
      freeList_ = node->next;
      return reinterpret_cast<T*>(node);
    }
    // Allocate new block if current is exhausted
    if (currentIdx_ >= BlockSize) {
      Block* newBlock = static_cast<Block*>(::operator new(sizeof(Block)));
      newBlock->next = blockList_;
      blockList_ = newBlock;
      currentIdx_ = 0;
    }
    T* ptr = reinterpret_cast<T*>(blockList_->data + sizeof(T) * currentIdx_);
    ++currentIdx_;
    return ptr;
  }

  void deallocate(T* ptr) {
    FreeNode* node = reinterpret_cast<FreeNode*>(ptr);
    node->next = freeList_;
    freeList_ = node;
  }
};

// Usage in solver code:
static PoolAllocator<ScratchBuffer> scratchPool;

void SolverIDA::step() {
  ScratchBuffer* buf = scratchPool.allocate();
  // ... use buf ...
  scratchPool.deallocate(buf);
}
```

---

### P6. Reduce SUNDIALS N_Vector Copies

**Expected Speedup:** 1-3% (IDA variable-step solver only)
**Implementation Effort:** Low
**Risk Level:** Low
**Scope:** IDA (variable-step) solver only — not applicable to FixedTimeStep (SolverSIM/SolverTRAP)

> **Code Analysis (March 2026).** Upstream code analysis confirms this optimization is **IDA-specific**. The FixedTimeStep solver does not use the N_Vector copy paths that P6 targets. See the Code Analysis section below.

#### Description

The SUNDIALS IDA solver interface uses `N_Vector` objects to pass state vectors, derivative vectors, and residual vectors between the solver and user code. In the current Dynawo-SUNDIALS integration, there are several places where vector data is copied between `N_Vector` internal storage and Dynawo's own `std::vector<double>` containers. These copies occur at every solver call (residual evaluation, Jacobian evaluation), adding up to significant overhead for large systems.

Many of these copies can be eliminated by wrapping Dynawo's existing vectors as `N_Vector`s using SUNDIALS' custom vector operations interface, or by directly using `NV_DATA_S()` to access the `N_Vector` data pointer and passing it to Dynawo functions without intermediate copies. The SUNDIALS serial `N_Vector` stores its data as a contiguous `double*` array, so direct pointer access is safe and efficient.

This optimization has the lowest risk of any proposed change because it only affects the SUNDIALS interface layer, not the solver logic or model evaluation code. The main caution is ensuring that the `N_Vector` data is not reallocated or freed while Dynawo holds a pointer to it.

#### Code Analysis: FixedTimeStep vs IDA N_Vector Usage

**FixedTimeStep (SolverSIM/SolverTRAP):** Uses `std::vector<double>` (`vectorY_`, `vectorYp_`, `vectorYSave_`) with `.assign()` for all save/restore operations — not N_Vector copy operations. The `sundialsVectorY_` is created via `N_VMake_Serial` wrapping `vectorY_.data()` (zero-copy wrapper in `Solver::Impl::init()` at `DYNSolverImpl.cpp:129-137`). KINSOL algebraic solvers share `sundialsVectorY_` by reference (`SolverKINCommon.cpp:93`) or create their own zero-copy wrapper (`SolverKINAlgRestoration.cpp:179`). **No N_Vector copy operations occur in the FixedTimeStep pipeline.**

**IDA (variable-step):** Passes `sundialsVectorY_`/`sundialsVectorYp_` to IDA API calls (`IDASolve`, `IDAReInit`, `IDAGetConsistentIC`). IDA callbacks (`evalF`, `evalG`, `evalJ`) receive N_Vectors from SUNDIALS and extract raw pointers via `NV_DATA_S()`. The `copyContinuousVariables()` calls at lines 358, 420, 629, 657, 676 copy data from N_Vector raw pointers into `ModelMulti::yLocal_`/`ypLocal_` via `std::vector::assign()`. These are the copy operations P6 could eliminate.

**`copyContinuousVariables` in ModelMulti (`DYNModelMulti.cpp:332`):**
```cpp
void ModelMulti::copyContinuousVariables(const double* y, const double* yp) {
  yLocal_.assign(y, y + sizeY());
  ypLocal_.assign(yp, yp + sizeY());
}
```
This is a simple `std::assign` from `double*` to `std::vector<double>` — the same pattern in both solver paths. However, in FixedTimeStep, these are called from `std::vector` sources (no N_Vector involvement), while in IDA, they are called with N_Vector-extracted pointers.

**Conclusion:** P6 is relevant only when using the IDA variable-step solver. For the TRAISIM target (FixedTimeStep/SolverSIM for real-time simulation), this optimization provides no benefit.

#### Implementation Sketch

```cpp
// Before: copying N_Vector data to/from internal vectors
int residualCallback(double t, N_Vector yy, N_Vector yp, N_Vector rr, void* userData) {
  SolverData* data = static_cast<SolverData*>(userData);
  int N = data->size;

  // Unnecessary copy: N_Vector -> std::vector
  std::vector<double> y(N), yPrime(N), residual(N);
  double* yyData = NV_DATA_S(yy);
  double* ypData = NV_DATA_S(yp);
  std::copy(yyData, yyData + N, y.begin());
  std::copy(ypData, ypData + N, yPrime.begin());

  data->model->evalF(t, y.data(), yPrime.data(), residual.data());

  // Unnecessary copy: std::vector -> N_Vector
  double* rrData = NV_DATA_S(rr);
  std::copy(residual.begin(), residual.end(), rrData);

  return 0;
}

// After: direct pointer access, zero copies
int residualCallback(double t, N_Vector yy, N_Vector yp, N_Vector rr, void* userData) {
  SolverData* data = static_cast<SolverData*>(userData);

  // Direct access to N_Vector internal storage
  double* y      = NV_DATA_S(yy);
  double* yPrime = NV_DATA_S(yp);
  double* f      = NV_DATA_S(rr);

  // Pass pointers directly -- no copies
  data->model->evalF(t, y, yPrime, f);

  return 0;
}
```

---

### P7. Profile-Guided Optimization (PGO)

**Expected Speedup:** 5-10%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

Profile-Guided Optimization (PGO) is a compiler technique where the optimizer uses runtime profiling data from representative workloads to make better decisions about inlining, branch prediction hints, code layout, and register allocation. GCC and Clang both support PGO through a two-pass build process: first, the code is compiled with instrumentation that records branch frequencies and function call counts; second, the code is recompiled using this profile data to optimize the hot paths.

For Dynawo, PGO is particularly effective because the simulation workload has very stable hot paths (the solver loop, Jacobian evaluation, and linear solve dominate), and branch prediction in the model evaluation code follows consistent patterns across time steps. The compiler can use this information to lay out hot code contiguously (improving instruction cache behavior), inline the most frequently called functions, and optimize branch prediction for the common case.

The implementation effort is low because it only requires changes to the build system, not the source code. The risk is similarly low because PGO produces a standard optimized binary; the only overhead is the additional build step.

#### Implementation Sketch

```bash
# Step 1: Build with instrumentation
cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-fprofile-generate=/tmp/pgo-data" \
    -DCMAKE_C_FLAGS="-fprofile-generate=/tmp/pgo-data"

make -j$(nproc)

# Step 2: Run representative workloads to collect profile data
export DYNAWO_HOME=$(pwd)/install
$DYNAWO_HOME/myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
$DYNAWO_HOME/myEnvDynawo.sh jobs examples/DynaWaltz/IEEE14/IEEE14_GeneratorDisconnections/IEEE14.jobs
# Run additional representative cases for better coverage

# Step 3: Rebuild using collected profile data
cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-fprofile-use=/tmp/pgo-data -fprofile-correction" \
    -DCMAKE_C_FLAGS="-fprofile-use=/tmp/pgo-data -fprofile-correction"

make -j$(nproc)
```

**CMake integration option:**

```cmake
option(DYNAWO_PGO_GENERATE "Build with PGO instrumentation" OFF)
option(DYNAWO_PGO_USE "Build using PGO profile data" OFF)
set(DYNAWO_PGO_DIR "/tmp/dynawo-pgo" CACHE PATH "Directory for PGO profile data")

if(DYNAWO_PGO_GENERATE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate=${DYNAWO_PGO_DIR}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-generate=${DYNAWO_PGO_DIR}")
elseif(DYNAWO_PGO_USE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-use=${DYNAWO_PGO_DIR} -fprofile-correction")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-use=${DYNAWO_PGO_DIR} -fprofile-correction")
endif()
```

---

### P8. Link-Time Optimization (LTO)

**Expected Speedup:** 3-5%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

Link-Time Optimization (LTO) allows the compiler to perform whole-program optimization across translation unit boundaries. Without LTO, the compiler can only optimize within a single `.cpp` file; with LTO, it can inline functions defined in other files, eliminate dead code across the entire program, and optimize interprocedural data flow.

For Dynawo, LTO is especially beneficial because the codebase is organized into many small source files with well-defined interfaces. Functions like `SubModel::evalF()` are called from the solver but defined in separate translation units; with LTO, the compiler can inline these across the library boundary. Similarly, utility functions in the Common library that are called in tight loops can be inlined into their callers.

LTO is straightforward to enable and has minimal risk. The main drawback is increased link time (which can double or triple for large projects), but this only affects the build process, not the resulting binary's behavior. Some older toolchains have issues with LTO and shared libraries, but GCC 13+ and Clang 16+ handle this reliably.

#### Implementation Sketch

```cmake
# In the top-level CMakeLists.txt:
option(DYNAWO_LTO "Enable Link-Time Optimization" OFF)

if(DYNAWO_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT lto_supported OUTPUT lto_error)
  if(lto_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    message(STATUS "LTO enabled")
  else()
    message(WARNING "LTO not supported: ${lto_error}")
  endif()
endif()
```

Or via command-line flags:

```bash
cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-flto" \
    -DCMAKE_C_FLAGS="-flto" \
    -DCMAKE_EXE_LINKER_FLAGS="-flto" \
    -DCMAKE_SHARED_LINKER_FLAGS="-flto"
```

---

### P9. GPU Acceleration for KLU

> **High risk / Long term / Possibly out of scope.** GPU acceleration introduces a heavy dependency (CUDA/cuSOLVER) and is only beneficial above a large system-size crossover point. The data transfer overhead may negate gains for typical use cases.

**Expected Speedup:** 20-50% (for very large systems, 10,000+ equations)
**Implementation Effort:** High
**Risk Level:** High

#### Description

The KLU sparse direct solver from SuiteSparse is the primary linear solver in Dynawo's IDA integration. For large power system models (10,000+ equations), the sparse LU factorization and triangular solve phases dominate simulation time. GPU acceleration can offload these operations to massively parallel hardware, providing substantial speedup for the largest models.

There are two approaches: (1) Replace KLU with a GPU-accelerated sparse direct solver such as cuSOLVER (NVIDIA) or rocSOLVER (AMD), which provide sparse LU factorization on GPUs. (2) Keep KLU for the symbolic analysis and use GPU kernels for the numerical factorization and triangular solve phases. The second approach is more practical because KLU's symbolic analysis (ordering, elimination tree computation) is inherently sequential and fast, while the numerical phases are parallelizable.

The challenge is that sparse direct solvers have irregular data access patterns that do not map well to GPU architectures for small-to-medium matrices. The crossover point where GPU acceleration becomes beneficial is typically around 5,000-10,000 equations. For smaller systems, the data transfer overhead between CPU and GPU memory dominates. This optimization should be pursued only after confirming that linear solve is the bottleneck for the target use cases, and only for the largest models.

#### Implementation Sketch

```cpp
// GPU-accelerated linear solve using cuSOLVER
#ifdef DYNAWO_GPU_SOLVE
#include <cusolverSp.h>
#include <cuda_runtime.h>

class GPUSparseLinearSolver {
  cusolverSpHandle_t handle_;
  cusparseMatDescr_t descr_;

  // Device memory
  int* d_colPtr_;
  int* d_rowIdx_;
  double* d_values_;
  double* d_rhs_;
  double* d_solution_;

  int nnz_;
  int n_;
  bool allocated_;

public:
  GPUSparseLinearSolver() : allocated_(false) {
    cusolverSpCreate(&handle_);
    cusparseCreateMatDescr(&descr_);
    cusparseSetMatType(descr_, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatIndexBase(descr_, CUSPARSE_INDEX_BASE_ZERO);
  }

  ~GPUSparseLinearSolver() {
    if (allocated_) {
      cudaFree(d_colPtr_);
      cudaFree(d_rowIdx_);
      cudaFree(d_values_);
      cudaFree(d_rhs_);
      cudaFree(d_solution_);
    }
    cusolverSpDestroy(handle_);
    cusparseDestroyMatDescr(descr_);
  }

  void setup(int n, int nnz, const int* colPtr, const int* rowIdx) {
    n_ = n;
    nnz_ = nnz;
    cudaMalloc(&d_colPtr_, (n + 1) * sizeof(int));
    cudaMalloc(&d_rowIdx_, nnz * sizeof(int));
    cudaMalloc(&d_values_, nnz * sizeof(double));
    cudaMalloc(&d_rhs_, n * sizeof(double));
    cudaMalloc(&d_solution_, n * sizeof(double));

    // Copy structure (done once)
    cudaMemcpy(d_colPtr_, colPtr, (n + 1) * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rowIdx_, rowIdx, nnz * sizeof(int), cudaMemcpyHostToDevice);
    allocated_ = true;
  }

  int solve(const double* values, const double* rhs, double* solution) {
    // Copy numerical values (every solve)
    cudaMemcpy(d_values_, values, nnz_ * sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rhs_, rhs, n_ * sizeof(double), cudaMemcpyHostToDevice);

    // Sparse LU solve on GPU
    int singularity = -1;
    cusolverSpDcsrlsvluHost(handle_, n_, nnz_, descr_,
                             d_values_, d_colPtr_, d_rowIdx_,
                             d_rhs_, 1e-14, 0, d_solution_, &singularity);

    // Copy solution back
    cudaMemcpy(solution, d_solution_, n_ * sizeof(double), cudaMemcpyDeviceToHost);

    return (singularity >= 0) ? -1 : 0;
  }
};
#endif
```

---

## Algorithmic Optimizations

### A1. Adaptive Factorization Control

**Expected Speedup:** 5-8%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

In the current solver implementation, the Jacobian matrix is factorized (symbolic + numerical LU decomposition) on a fixed schedule or whenever the Newton solver requests a new Jacobian. The symbolic factorization phase, which computes the fill-reducing ordering and the elimination tree, is particularly expensive but only needs to be repeated when the sparsity structure of the Jacobian changes (e.g., after a topology change due to a line trip or generator disconnection).

Adaptive factorization control tracks whether the Jacobian structure has actually changed since the last symbolic factorization. If the structure is unchanged (same non-zero pattern), only numerical refactorization is performed, which is 2-5x faster than a full symbolic+numerical factorization. The decision to perform symbolic factorization is based on a "structure changed" flag that is set by the model evaluation code whenever the network topology or model connections change.

This optimization is low-risk because it preserves the existing factorization quality -- it simply skips redundant work. The primary requirement is a reliable mechanism for detecting structure changes, which can be implemented by tracking the `nnz` count and a hash of the column pointer array.

#### Developer Feedback (TRAISIM Discussion)

Gautier Bureau confirmed that the excessive symbolic factorization is a known issue. In RTE's large-scale French system tests, they do not observe as many factorizations because their scenarios tend to have fewer cascading events. However, for the TRAISIM use case (300,000-variable system with cascading automata), the problem is acute: profiling showed ~30% of computation time spent on symbolic factorization alone.

Gautier's key insight: the root cause is that Dynawo's `modeChangeType_t` heuristic in the Modelica-to-C++ translation flags `ALGEBRAIC_J_UPDATE_MODE` (which triggers full Jacobian/factorization update) for **any** Boolean variable change in an `if` clause. This was originally intended to catch the `running` Boolean (generator on/off), but it also fires for many minor control automata actions (e.g., OEL activation, tap changer steps). The result is that even small events during the post-fault recovery period (e.g., 7 seconds of automata actions after a generator trip) force repeated full factorizations.

Gautier suggested a two-pronged approach:
1. **Metrics-based (solver-side):** The monitoring approach described below (structure hash, nnz tracking) — applicable immediately.
2. **Engineering-based (model-side):** A new event severity classification in the model translation layer (see A7) that provides richer information to the solver about whether a change is "severe" enough to warrant factorization. This approach gives better results because it uses domain knowledge about what events actually affect the Jacobian structure significantly.

The RAMSES simulator (used by Petros Aristidou) takes the most aggressive approach: it only forces Jacobian update for short circuits. For all other events, it relies on Newton iteration failure as a fallback trigger — if the stale Jacobian causes more than 5 Newton iterations, factorization is forced. This is too aggressive for offline accuracy but demonstrates the headroom available for real-time applications.

#### Implementation Sketch

```cpp
class AdaptiveFactorizationController {
  bool structureChanged_;
  int lastNnz_;
  uint64_t lastStructureHash_;

public:
  AdaptiveFactorizationController()
      : structureChanged_(true), lastNnz_(0), lastStructureHash_(0) {}

  // Called by model code when topology changes
  void markStructureChanged() {
    structureChanged_ = true;
  }

  // Called before factorization to decide which type to perform
  enum FactorizationType { FULL_SYMBOLIC, NUMERICAL_ONLY };

  FactorizationType decideFactorizationType(
      int nnz, const int* colPtr, int nCols) {

    if (structureChanged_) {
      structureChanged_ = false;
      lastNnz_ = nnz;
      lastStructureHash_ = computeHash(colPtr, nCols + 1);
      return FULL_SYMBOLIC;
    }

    // Quick check: same nnz?
    if (nnz != lastNnz_) {
      lastNnz_ = nnz;
      lastStructureHash_ = computeHash(colPtr, nCols + 1);
      return FULL_SYMBOLIC;
    }

    // Deeper check: same structure hash?
    uint64_t currentHash = computeHash(colPtr, nCols + 1);
    if (currentHash != lastStructureHash_) {
      lastStructureHash_ = currentHash;
      return FULL_SYMBOLIC;
    }

    return NUMERICAL_ONLY;
  }

private:
  uint64_t computeHash(const int* data, int len) {
    // FNV-1a hash
    uint64_t hash = 14695981039346656037ULL;
    for (int i = 0; i < len; ++i) {
      hash ^= static_cast<uint64_t>(data[i]);
      hash *= 1099511628211ULL;
    }
    return hash;
  }
};
```

---

### A2. Matrix Structure Change Tolerance

**Expected Speedup:** 3-5%
**Implementation Effort:** Low
**Risk Level:** Medium (operational debug risk is elevated without robust fallback and telemetry — a stale symbolic factorization produces silent accuracy degradation that is difficult to diagnose)

#### Description

Related to adaptive factorization control, this optimization introduces a tolerance for what constitutes a "structural change" in the Jacobian. In some simulation scenarios, small perturbations (e.g., a very small admittance element appearing or disappearing) change the sparsity pattern technically but do not significantly affect the quality of the existing factorization. By treating small structural changes below a threshold as non-structural, the solver can continue using numerical-only refactorization.

The tolerance is defined in terms of the number of changed non-zero entries relative to the total. If fewer than a configurable percentage (e.g., 1%) of non-zero positions change, the existing symbolic factorization is reused. The risk is that reusing a stale symbolic factorization may lead to increased fill-in or numerical instability, so a fallback mechanism is needed: if the numerical refactorization produces a poor-quality result (measured by the KLU condition number estimate or Newton convergence degradation), a full re-factorization is triggered.

This optimization works best in conjunction with A1 (adaptive factorization control) and A3 (KLU numerical-only refactorization), forming a layered factorization strategy.

#### Implementation Sketch

```cpp
class StructureChangeTolerance {
  double tolerance_;         // fraction of nnz that can change
  int maxChangedEntries_;    // absolute cap

  std::vector<int> lastRowIndices_;   // stored row indices from last factorization
  int lastNnz_;

public:
  StructureChangeTolerance(double tol = 0.01, int maxChanged = 50)
      : tolerance_(tol), maxChangedEntries_(maxChanged), lastNnz_(0) {}

  bool isStructurallyEquivalent(
      const int* rowIdx, const int* colPtr, int nCols, int nnz) {

    if (lastNnz_ == 0) {
      storeStructure(rowIdx, colPtr, nCols, nnz);
      return false;  // first time: need full factorization
    }

    // Count changed entries
    int changed = 0;
    if (nnz != lastNnz_) {
      changed = std::abs(nnz - lastNnz_);
    } else {
      for (int i = 0; i < nnz && changed <= maxChangedEntries_; ++i) {
        if (rowIdx[i] != lastRowIndices_[i])
          ++changed;
      }
    }

    int threshold = std::min(
        maxChangedEntries_,
        static_cast<int>(tolerance_ * lastNnz_));

    if (changed <= threshold) {
      return true;   // close enough, reuse symbolic
    }

    storeStructure(rowIdx, colPtr, nCols, nnz);
    return false;    // too many changes, need full factorization
  }

private:
  void storeStructure(const int* rowIdx, const int* colPtr, int nCols, int nnz) {
    lastNnz_ = nnz;
    lastRowIndices_.assign(rowIdx, rowIdx + nnz);
  }
};
```

---

### A3. KLU Numerical-Only Refactorization

**Expected Speedup:** 5-7%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

KLU provides separate API calls for symbolic analysis (`klu_analyze`), numerical factorization (`klu_factor`), and numerical refactorization (`klu_refactor`). The `klu_refactor` function reuses the existing symbolic information (elimination tree, column ordering, memory layout) and only recomputes the numerical values of L and U. This is significantly faster than `klu_factor`, which also allocates memory and initializes data structures.

In the current Dynawo-KLU integration, it is possible that `klu_factor` is called more often than necessary, even when the sparsity structure has not changed. By explicitly tracking whether a prior symbolic factorization exists and calling `klu_refactor` instead of `klu_factor` when appropriate, substantial time savings are achieved. The `klu_refactor` path is typically 2-3x faster than `klu_factor` for the same matrix.

This optimization pairs naturally with A1 (adaptive factorization control). When A1 determines that the structure is unchanged, this optimization ensures that `klu_refactor` is called instead of `klu_factor`. The risk is low because KLU's `klu_refactor` is a well-tested code path designed for exactly this use case.

#### Implementation Sketch

```cpp
class KLUFactorizationManager {
  klu_symbolic* symbolic_;
  klu_numeric* numeric_;
  klu_common common_;
  bool hasSymbolic_;
  bool hasNumeric_;

public:
  KLUFactorizationManager() : symbolic_(nullptr), numeric_(nullptr),
                                hasSymbolic_(false), hasNumeric_(false) {
    klu_defaults(&common_);
  }

  ~KLUFactorizationManager() {
    if (numeric_) klu_free_numeric(&numeric_, &common_);
    if (symbolic_) klu_free_symbolic(&symbolic_, &common_);
  }

  int factorize(int n, const int* Ap, const int* Ai, const double* Ax,
                bool structureChanged) {

    if (structureChanged || !hasSymbolic_) {
      // Full symbolic + numerical factorization
      if (symbolic_) klu_free_symbolic(&symbolic_, &common_);
      if (numeric_) klu_free_numeric(&numeric_, &common_);

      symbolic_ = klu_analyze(n, const_cast<int*>(Ap),
                              const_cast<int*>(Ai), &common_);
      if (!symbolic_) return -1;
      hasSymbolic_ = true;

      numeric_ = klu_factor(const_cast<int*>(Ap), const_cast<int*>(Ai),
                            const_cast<double*>(Ax), symbolic_, &common_);
      hasNumeric_ = (numeric_ != nullptr);
      return hasNumeric_ ? 0 : -1;
    }

    if (hasNumeric_) {
      // Numerical-only refactorization (2-3x faster)
      int status = klu_refactor(const_cast<int*>(Ap), const_cast<int*>(Ai),
                                const_cast<double*>(Ax), symbolic_,
                                numeric_, &common_);
      if (status == 0) {
        // Refactorization failed (e.g., singular), try full factorization
        klu_free_numeric(&numeric_, &common_);
        numeric_ = klu_factor(const_cast<int*>(Ap), const_cast<int*>(Ai),
                              const_cast<double*>(Ax), symbolic_, &common_);
        hasNumeric_ = (numeric_ != nullptr);
        return hasNumeric_ ? 0 : -1;
      }
      return 0;
    }

    // First numerical factorization after symbolic
    numeric_ = klu_factor(const_cast<int*>(Ap), const_cast<int*>(Ai),
                          const_cast<double*>(Ax), symbolic_, &common_);
    hasNumeric_ = (numeric_ != nullptr);
    return hasNumeric_ ? 0 : -1;
  }

  int solve(int n, int nrhs, double* B) {
    if (!hasNumeric_) return -1;
    klu_solve(symbolic_, numeric_, n, nrhs, B, &common_);
    return 0;
  }
};
```

---

### A4. Improved COLAMD Ordering

**Expected Speedup:** 2-4%
**Implementation Effort:** Medium
**Risk Level:** Low

#### Description

KLU uses the COLAMD (Column Approximate Minimum Degree) algorithm by default for computing a fill-reducing column ordering before LU factorization. While COLAMD is a good general-purpose ordering, power system Jacobian matrices have specific structure (block-bordered diagonal, with coupling through the network admittance matrix) that can be exploited by specialized ordering strategies.

Two improvements are possible: (1) Use AMD (Approximate Minimum Degree) ordering on the symmetric structure A+A^T, which sometimes produces better orderings for power system matrices. KLU supports this via the `klu_common.ordering` parameter. (2) Apply a nested dissection ordering using METIS or Scotch, which is better for very large matrices (5000+ rows) because it exploits the graph structure of the power network, which has small separators due to the physical topology.

The implementation involves trying multiple ordering strategies on representative test matrices and selecting the one with the lowest estimated fill-in. This can be done offline (choosing a fixed ordering strategy) or adaptively (trying multiple orderings on the first factorization and caching the best one).

#### Implementation Sketch

```cpp
class OrderingSelector {
public:
  enum Strategy {
    COLAMD,   // KLU default
    AMD,      // Symmetric AMD
    METIS,    // Nested dissection (requires METIS library)
    AUTO      // Try all, pick best
  };

  static int selectOrdering(klu_common* common, int n,
                            const int* Ap, const int* Ai,
                            Strategy strategy) {
    switch (strategy) {
      case COLAMD:
        common->ordering = 0;  // KLU default: COLAMD
        break;
      case AMD:
        common->ordering = 1;  // AMD ordering
        break;
      case METIS:
        // METIS ordering requires KLU compiled with METIS support
        common->ordering = 2;  // METIS nested dissection
        break;
      case AUTO: {
        // Try each ordering and estimate fill-in
        int bestOrdering = 0;
        int bestFillIn = INT_MAX;

        for (int ord = 0; ord <= 2; ++ord) {
          klu_common testCommon;
          klu_defaults(&testCommon);
          testCommon.ordering = ord;

          klu_symbolic* sym = klu_analyze(n,
              const_cast<int*>(Ap), const_cast<int*>(Ai), &testCommon);
          if (sym) {
            int fillIn = static_cast<int>(sym->lnz + sym->unz);
            if (fillIn < bestFillIn) {
              bestFillIn = fillIn;
              bestOrdering = ord;
            }
            klu_free_symbolic(&sym, &testCommon);
          }
        }

        common->ordering = bestOrdering;
        break;
      }
    }
    return 0;
  }
};
```

---

### A5. Partial Jacobian Updates

> **High risk / Long term / Possibly out of scope.** This optimization carries significant implementation complexity and may not be needed if Phase 0 factorization avoidance (A7, A1, A2, A3) provides sufficient speedup.

**Expected Speedup:** 10-20%
**Implementation Effort:** High
**Risk Level:** Medium-High

#### Description

In a multi-model simulation, the global Jacobian matrix is block-structured: each submodel contributes a diagonal block, and the network model provides coupling blocks. When only a subset of submodels have changed state significantly (e.g., a generator hitting a limit while others are in steady state), only the corresponding diagonal blocks need to be recomputed. The unchanged blocks can be reused from the previous evaluation.

Implementing partial Jacobian updates requires: (1) A change-detection mechanism that identifies which submodels have experienced significant state changes since the last Jacobian evaluation. This can be based on the norm of the state variable change vector for each submodel, compared to a threshold. (2) A Jacobian assembly framework that can selectively update blocks while preserving the rest of the matrix. (3) A refactorization strategy that accounts for the partially-updated matrix (typically, numerical refactorization is sufficient if only values changed, not structure).

The potential speedup is large because Jacobian evaluation is typically the most expensive phase, and in many simulations only a small fraction of submodels are actively changing at any given time. However, the implementation is complex and carries medium risk because an incorrectly reused Jacobian block can lead to Newton convergence failures or incorrect results.

#### Developer Feedback (TRAISIM Discussion)

Gautier Bureau assessed this as conceptually straightforward but "difficult to implement" in practice, particularly with the existing sparse matrix data structures. Updating already-factorized LU factors after partial Jacobian changes is non-trivial. Literature review found a few papers demonstrating the mathematics, but only on small matrices in prototype code, not at production scale.

**Recommendation:** Prioritize A1, A2, A3, and A7 (event severity classification) first. These address the same root problem (unnecessary Jacobian work during events) with much lower implementation complexity. If those optimizations reduce Jacobian overhead sufficiently, A5 may not be needed. If further Jacobian savings are still required after implementing the factorization control improvements, A5 should be prototyped on a small test case before committing to a full implementation.

#### Implementation Sketch

```cpp
class PartialJacobianUpdater {
  struct SubModelState {
    std::vector<double> lastState;    // state at last Jacobian eval
    double changeTolerance;           // threshold for re-evaluation
    bool needsUpdate;
  };

  std::vector<SubModelState> subModelStates_;

public:
  void init(int nSubModels, double tolerance = 1e-4) {
    subModelStates_.resize(nSubModels);
    for (auto& s : subModelStates_) {
      s.changeTolerance = tolerance;
      s.needsUpdate = true;  // always evaluate first time
    }
  }

  // Determine which submodels need Jacobian re-evaluation
  void detectChanges(const std::vector<SubModel*>& subModels,
                     const double* currentState,
                     const std::vector<int>& offsets,
                     const std::vector<int>& sizes) {
    for (size_t i = 0; i < subModels.size(); ++i) {
      const double* y = currentState + offsets[i];
      int n = sizes[i];

      if (subModelStates_[i].lastState.empty()) {
        subModelStates_[i].lastState.assign(y, y + n);
        subModelStates_[i].needsUpdate = true;
        continue;
      }

      // Compute relative change norm
      double changeNorm = 0.0;
      double stateNorm = 0.0;
      for (int j = 0; j < n; ++j) {
        double diff = y[j] - subModelStates_[i].lastState[j];
        changeNorm += diff * diff;
        stateNorm += y[j] * y[j];
      }
      changeNorm = std::sqrt(changeNorm);
      stateNorm = std::sqrt(stateNorm);

      double relChange = (stateNorm > 0) ? changeNorm / stateNorm : changeNorm;
      subModelStates_[i].needsUpdate = (relChange > subModelStates_[i].changeTolerance);

      if (subModelStates_[i].needsUpdate) {
        subModelStates_[i].lastState.assign(y, y + n);
      }
    }
  }

  bool needsUpdate(int subModelIdx) const {
    return subModelStates_[subModelIdx].needsUpdate;
  }

  int countUpdates() const {
    int count = 0;
    for (const auto& s : subModelStates_) {
      if (s.needsUpdate) ++count;
    }
    return count;
  }
};

// Usage in ModelMulti::evalJt:
void ModelMulti::evalJt(double t, double cj, SparseMatrix& Jt) {
  partialUpdater_.detectChanges(subModels_, currentState_, offsets_, sizes_);

  int nUpdated = partialUpdater_.countUpdates();
  int nTotal = static_cast<int>(subModels_.size());

  // Only re-evaluate changed submodel blocks
  for (int i = 0; i < nTotal; ++i) {
    if (partialUpdater_.needsUpdate(i)) {
      subModels_[i]->evalJt(t, cj, Jt, offsets_[i]);
    }
    // Unchanged blocks remain in Jt from previous evaluation
  }

  // Always re-evaluate network coupling blocks (they depend on all states)
  networkModel_->evalJt(t, cj, Jt);
}
```




---

### A6. Schur Complement Decomposition

> **High risk / Long term / Possibly out of scope.** Requires significant architectural changes and is only beneficial for the largest systems. Should be evaluated as a research prototype before any production commitment.

**Expected Speedup:** 15-30% (for large multi-machine systems)
**Implementation Effort:** High
**Risk Level:** High

#### Description

Power system simulation models naturally decompose into network variables (bus voltages and currents) and device variables (generator internal states, controller states). The Jacobian matrix reflects this structure: device equations only couple to the network through their terminal bus, while network equations couple to all connected devices. This block structure can be exploited through Schur complement decomposition.

The idea is to partition the state vector into network variables (y_n) and device variables (y_d). The Jacobian has the block form:

```
J = | J_nn  J_nd |
    | J_dn  J_dd |
```

where J_dd is block-diagonal (devices do not directly couple to each other). The Schur complement S = J_nn - J_nd * J_dd^{-1} * J_dn is a much smaller dense or sparse matrix that captures the reduced network behavior. By factoring J_dd (cheap, block-diagonal) and S (small) separately, the overall linear solve is faster than factoring the full J.

For a system with N_n network variables and N_d device variables where N_d >> N_n, the speedup can be substantial: factoring J_dd is O(sum of n_i^3) for device blocks of size n_i (typically small, 3-15 each), and factoring S is O(N_n^3) or less with sparsity. The full factorization would be O((N_n + N_d)^2 * fill).

The risk is high because the Schur complement approach changes the linear solver fundamentally and requires careful handling of the partitioning, especially during topology changes when the network structure changes. It also requires that the J_dd blocks are well-conditioned for independent factorization.

#### Implementation Sketch

```cpp
class SchurComplementSolver {
  // Partition information
  int nNetwork_;     // number of network variables
  int nDevice_;      // number of device variables
  int nTotal_;

  // Device block factorizations (block-diagonal)
  struct DeviceBlock {
    int offset;
    int size;
    klu_symbolic* sym;
    klu_numeric* num;
    klu_common common;
  };
  std::vector<DeviceBlock> deviceBlocks_;

  // Schur complement matrix (dense for simplicity, sparse for large networks)
  std::vector<double> schurMatrix_;      // N_n x N_n
  std::vector<int> schurPivots_;         // LU pivots for LAPACK

  // Coupling matrices (sparse)
  SparseMatrix Jnd_;  // network-device coupling
  SparseMatrix Jdn_;  // device-network coupling

public:
  // Factor the system using Schur complement
  int factorize(const SparseMatrix& J, const Partition& part) {
    nNetwork_ = part.networkSize();
    nDevice_ = part.deviceSize();
    nTotal_ = nNetwork_ + nDevice_;

    // 1. Factor each device diagonal block (parallel)
    #pragma omp parallel for if(deviceBlocks_.size() > 4)
    for (size_t i = 0; i < deviceBlocks_.size(); ++i) {
      DeviceBlock& b = deviceBlocks_[i];
      // Extract and factor J_dd block i
      extractAndFactor(J, b);
    }

    // 2. Extract coupling matrices J_nd and J_dn
    extractCoupling(J, part, Jnd_, Jdn_);

    // 3. Compute Schur complement: S = J_nn - J_nd * J_dd^{-1} * J_dn
    extractNetworkBlock(J, part, schurMatrix_);
    computeSchurComplement(schurMatrix_);

    // 4. Factor the Schur complement (small, dense LU)
    schurPivots_.resize(nNetwork_);
    int info;
    dgetrf_(&nNetwork_, &nNetwork_, schurMatrix_.data(), &nNetwork_,
            schurPivots_.data(), &info);

    return (info == 0) ? 0 : -1;
  }

  // Solve using Schur complement:
  // [J_nn J_nd] [x_n]   [b_n]
  // [J_dn J_dd] [x_d] = [b_d]
  int solve(double* b, double* x) {
    // 1. Solve J_dd * z_d = b_d for each device block
    std::vector<double> zd(nDevice_);
    #pragma omp parallel for if(deviceBlocks_.size() > 4)
    for (size_t i = 0; i < deviceBlocks_.size(); ++i) {
      solveDeviceBlock(deviceBlocks_[i], b, zd.data());
    }

    // 2. Form reduced RHS: r = b_n - J_nd * z_d
    std::vector<double> r(nNetwork_);
    std::copy(b, b + nNetwork_, r.begin());
    Jnd_.matvec(-1.0, zd.data(), 1.0, r.data());

    // 3. Solve S * x_n = r (using factored Schur complement)
    std::copy(r.begin(), r.end(), x);
    int nrhs = 1, info;
    dgetrs_("N", &nNetwork_, &nrhs, schurMatrix_.data(), &nNetwork_,
            schurPivots_.data(), x, &nNetwork_, &info);

    // 4. Back-substitute: x_d = z_d - J_dd^{-1} * J_dn * x_n
    std::vector<double> temp(nDevice_);
    Jdn_.matvec(1.0, x, 0.0, temp.data());
    #pragma omp parallel for if(deviceBlocks_.size() > 4)
    for (size_t i = 0; i < deviceBlocks_.size(); ++i) {
      solveDeviceBlock(deviceBlocks_[i], temp.data(), x + nNetwork_);
    }
    for (int i = 0; i < nDevice_; ++i) {
      x[nNetwork_ + i] = zd[i] - x[nNetwork_ + i];
    }

    return 0;
  }
};
```


---

### A7. Event Severity Classification for Reinit/Factorization Control

**Expected Speedup:** 10-15% (during event-heavy periods; compounds with A1/A2/A3)
**Implementation Effort:** Low — solver infrastructure already in place; changes are model-layer only
**Risk Level:** Low
**Priority:** Highest — this is the single most impactful optimization identified during profiling and developer consultation
**Source:** TRAISIM project discussion with Gautier Bureau (former Dynawo lead developer, RTE)

#### Description

The current `modeChangeType_t` enum in Dynawo classifies events into four levels: `NO_MODE`, `DIFFERENTIAL_MODE`, `ALGEBRAIC_MODE`, and `ALGEBRAIC_J_UPDATE_MODE`. The Modelica-to-C++ translation layer assigns `ALGEBRAIC_J_UPDATE_MODE` to any model equation that involves a Boolean variable in an `if` clause. This heuristic was designed to catch the `running` Boolean (generator on/off) but triggers for many minor events (OEL activations, tap changer steps, AVR limit actions). The solver treats `ALGEBRAIC_J_UPDATE_MODE` as a signal to perform a full Jacobian update and symbolic refactorization, even when the event has negligible impact on the system Jacobian.

Gautier Bureau prototyped a solution for the IDA solver: adding a new, higher-severity flag to the `modeChangeType_t` enum that is triggered only for truly disruptive events (short circuits via `NodeFault` model, and generator disconnections via the `running` Boolean). Minor events would receive the standard `ALGEBRAIC_J_UPDATE_MODE` flag, which allows the solver to skip full symbolic refactorization.

The change in the Modelica translation layer is small (~3 lines of code in the C++ code generation). The new flag is derived from heuristic name matching: if the model is a `NodeFault` model or the Boolean variable is named `running`, the severe flag is set. All other Boolean-driven mode changes get the standard `ALGEBRAIC_J_UPDATE_MODE` flag.

#### Code Analysis: FixedTimeStep Solver Already Supports Event Severity

A detailed analysis of the upstream codebase confirms that **the FixedTimeStep solver (SolverSIM/SolverTRAP) already fully implements event severity handling.** No solver-side changes are needed — the work is entirely at the model classification layer.

The event severity classification system was introduced by Adrien Guironnet (RTE) in commit `9ada292` (issue #346, July 2019) and enhanced by Florentine Rosiere in commit `bf791ae` (issue #947, July 2020, adding the configurable `minimumModeChangeTypeForAlgebraicRestoration` parameter). Both the IDA and FixedTimeStep solvers share the same base class (`Solver::Impl`) and identical severity-handling infrastructure.

The FixedTimeStep solver has three key code paths that already respond to event severity:

1. **`handleRoot()` (`DYNSolverCommonFixedTimeStep.cpp:396-404`):** Sets `factorizationForced_=true` only for `ALGEBRAIC_J_UPDATE_MODE`. For lower-severity events, it skips factorization and even grows the time step.

2. **`reinit()` (`DYNSolverCommonFixedTimeStep.cpp:448-518`):** Checks `modeChangeType < minimumModeChangeTypeForAlgebraicRestoration_` and skips algebraic restoration entirely for low-severity events.

3. **`setupNewAlgRestoration()` (`DYNSolverCommonFixedTimeStep.cpp:420-442`):** Uses different KINSOL tolerance sets depending on severity — tighter tolerances for `ALGEBRAIC_J_UPDATE_MODE` (forced J, `msbsetAlgJ_=1`), relaxed tolerances for `ALGEBRAIC_MODE` (`msbsetAlg_=5`).

4. **`callAlgebraicSolver()` (`DYNSolverCommonFixedTimeStep.cpp:309-332`):** Uses the `factorizationForced_` flag to decide whether to force Jacobian setup at the next Newton solve.

The configurable parameter `minimumModeChangeTypeForAlgebraicRestoration` (default: `ALGEBRAIC_MODE`) is already available in solver PAR files for both IDA and FixedTimeStep solvers.

**Comparison: IDA vs FixedTimeStep handling:**

| Aspect | IDA Solver | FixedTimeStep |
|--------|-----------|---------------|
| Mode classification | Same `modeChangeType_t` enum | Same |
| `setupNewAlgRestoration()` | Returns `true` (force J) for `ALGEBRAIC_J_UPDATE_MODE` | Identical logic |
| `reinit()` | Checks `minimumModeChangeTypeForAlgebraicRestoration_` | Same check |
| KINSOL tolerance sets | Dual sets (`*Alg_` vs `*AlgJ_`) | Same dual sets |
| Root handling | Triggers `IDAReInit` after reinit | Uses `factorizationForced_` flag |
| Factorization control | Implicit via IDA's internal Newton | Explicit `factorizationForced_` flag |

**Implication:** Any change to how models report their `modeChangeType_t` in `evalMode()` immediately benefits both solvers. Adding a new `ALGEBRAIC_J_UPDATE_SEVERE_MODE` level works automatically — the FixedTimeStep solver will only force factorization for the new severe level, while downgraded events skip factorization.

#### Why This Matters for Real-Time Performance

In TRAISIM profiling of the 300,000-variable system, ~30% of computation time during event periods was spent on symbolic factorization triggered by minor automata actions. A generator trip at t=0 correctly triggers factorization, but the subsequent 7 seconds of follow-up control actions (OELs, AVRs, secondary frequency control) each trigger unnecessary full factorizations. With event severity classification, only the initial generator trip would force full factorization; the follow-up events would use numerical-only refactorization or be deferred to Newton failure fallback.

This compounds with A1/A2/A3: event severity classification provides the model-side intelligence, while A1/A2/A3 provide the solver-side intelligence. Together they form a complete factorization control strategy.

**Estimated savings for FixedTimeStep:** In simulations with frequent low-severity events (e.g., OLTC tap changes every few seconds), this avoids a KLU symbolic + numeric factorization at each event. For large networks (10k+ buses), KLU factorization can be 10-30% of total solver time per step. If tap changes occur at ~100 of ~1000 total events, reclassifying them could save 5-15% of total simulation time.

#### Implementation Sketch

Since the solver infrastructure is already in place, the implementation focuses on the model classification layer:

```cpp
// Step 1: Extend the modeChangeType_t enum (DYNEnumUtils.h)
typedef enum {
  NO_MODE = 0,
  DIFFERENTIAL_MODE,
  ALGEBRAIC_MODE,
  ALGEBRAIC_J_UPDATE_MODE,
  ALGEBRAIC_J_UPDATE_SEVERE_MODE  // NEW: only for faults and disconnections
} modeChangeType_t;

// Step 2: In the Modelica compiler (dataContainer.py:2179-2208),
// refine the classification for discrete variable mode changes:
//   - Boolean named "running" → ALGEBRAIC_J_UPDATE_SEVERE_MODE
//   - All other Boolean-driven mode changes → ALGEBRAIC_J_UPDATE_MODE
// Currently, line 2208 emits: return ALGEBRAIC_J_UPDATE_MODE;
// For the "running" Boolean, change to: return ALGEBRAIC_J_UPDATE_SEVERE_MODE;

// Step 3: In ModelNetwork::evalMode() (DYNModelNetwork.cpp:1032-1063),
// refine the classification for network events:
//   - Topology changes (line trips, bus faults) → ALGEBRAIC_J_UPDATE_SEVERE_MODE
//   - State changes (tap steps, load shedding) → ALGEBRAIC_MODE (already the case)

// Step 4: No solver changes needed — both IDA and FixedTimeStep solvers
// already distinguish all modeChangeType_t levels through:
//   - handleRoot() → factorizationForced_ only for highest severity
//   - reinit() → minimumModeChangeTypeForAlgebraicRestoration_ gate
//   - setupNewAlgRestoration() → different KINSOL tolerances per severity
```

#### Relationship to Upstream Implementation

The event severity classification system was introduced upstream by Adrien Guironnet (commit `9ada292`, issue #346) and Florentine Rosiere (commit `bf791ae`, issue #947). The existing four-level enum and all solver-side handling for both IDA and FixedTimeStep are already in the `master` branch. Gautier Bureau's prototype adds the fifth level (`ALGEBRAIC_J_UPDATE_SEVERE_MODE`) and the model-layer reclassification — this is the only remaining work.

---

## Phased Roadmap

### Phase 0: Quick Wins

**Objective:** Achieve 15-25% overall speedup with minimal code changes and low risk, focusing on factorization avoidance.

**Items:**
- **A7. Event Severity Classification** (15-30% reduction in event-period time) — **Highest priority.** Code analysis confirms that the FixedTimeStep solver (SolverSIM/SolverTRAP) already fully supports event severity through shared infrastructure with IDA (`handleRoot()`, `reinit()`, `setupNewAlgRestoration()`). No solver-side changes are needed — only model-layer reclassification (adding `ALGEBRAIC_J_UPDATE_SEVERE_MODE` to the enum and updating `ModelNetwork::evalMode()` and the Modelica compiler). This reduces effort from Medium to Low.
- **A1. Adaptive Factorization Control** (5-8% speedup)
- **A2. Matrix Structure Change Tolerance** (3-5% speedup)
- **A3. KLU Numerical-Only Refactorization** (5-7% speedup)

**Rationale:** A7 is the single highest-impact quick win identified during the TRAISIM discussion with Gautier Bureau. Profiling shows that ~30% of event-period computation time is spent on symbolic factorizations triggered by minor automata events (tap changers, OELs) that do not actually change Jacobian structure. Code analysis of the upstream codebase (commits `9ada292` by Adrien Guironnet and `bf791ae` by Florentine Rosiere) confirms the four-level `modeChangeType_t` enum and all solver-side handling are already shared between IDA and FixedTimeStep solvers. The FixedTimeStep `handleRoot()` already distinguishes `ALGEBRAIC_J_UPDATE_MODE` from lower severities, and `reinit()` uses the configurable `minimumModeChangeTypeForAlgebraicRestoration` parameter — no solver-side port is needed. The remaining work is model-layer only: adding a fifth enum level and reclassifying events in `ModelNetwork::evalMode()` and the Modelica compiler (`dataContainer.py`). A1, A2, and A3 complement A7 by providing layered factorization control: A1 adds decision logic, A2 extends skip criteria, and A3 ensures the fast `klu_refactor` path is used when symbolic refactorization is skipped.

**Tasks:**
1. Add `ALGEBRAIC_J_UPDATE_SEVERE_MODE` to the `modeChangeType_t` enum (`DYNEnumUtils.h`). No solver-side changes needed — the FixedTimeStep solver already handles severity levels through `handleRoot()`, `reinit()`, and `setupNewAlgRestoration()`.
2. Update `ModelNetwork::evalMode()` to emit `ALGEBRAIC_J_UPDATE_SEVERE_MODE` for topology changes (line trips, bus faults) instead of `ALGEBRAIC_J_UPDATE_MODE`. State changes (tap steps) remain at `ALGEBRAIC_MODE`.
3. Update the Modelica compiler (`dataContainer.py:2208`) to emit `ALGEBRAIC_J_UPDATE_SEVERE_MODE` only for the `running` Boolean; all other Boolean-driven mode changes remain at `ALGEBRAIC_J_UPDATE_MODE`.
4. Instrument the current factorization code to measure symbolic vs. numerical factorization frequency.
5. Implement A1 (adaptive factorization controller with structure hash).
6. Implement A3 (explicit `klu_refactor` path) alongside A1.
7. Implement A2 (structure change tolerance) with configurable threshold.
8. Benchmark on Nordic test system and a large-scale test case, measuring factorization skip rate and event-period speedup.
9. Tune severity classification and tolerance parameters based on benchmarks.
10. Run the full test suite to verify correctness — ensure Newton convergence fallback triggers correctly when severity is underestimated.

**Go/No-Go Criteria for Phase 1:**
- At least 10% measured speedup on the large-scale test case.
- Event-period symbolic factorization rate reduced by at least 50%.
- No increase in Newton iteration failures or convergence warnings.
- All unit tests and integration tests pass.

---

### Phase 1: Medium Effort

**Objective:** Achieve an additional 10-20% speedup through remaining data structure improvements, Jacobian assembly optimization, and careful exploration of partial Jacobian updates.

**Items:**
- **P3 Phase A. Structural Index Map for Jacobian Extraction** — Build a compile-time structural index map from the first dense Adept Jacobian evaluation; skip known-zero entries in the extraction loop. Pre-allocate the `SparseMatrix` to avoid per-call heap allocation. This is a scoped prototype — Phase B/C (invalidation strategies) are gated by validation.
- **Remaining `std::map` audit** — Flat Vector Derivatives was implemented upstream by Gautier Bureau (commit `#3749`), but only for the Network model's `Derivatives` class. Audit and convert remaining `std::map<int, double>` usage in Modelica-generated C++ and SubModel coupling code.
- **A4. Improved COLAMD Ordering** — Cache and reuse ordering across factorizations.

**Rationale:** P3 Phase A targets the O(n²) extraction loop in `evalJtAdept`, which processes all `sizeF × sizeY` entries even though most are zero for typical Modelica submodels. The structural index map is safe (conservative: invalidated on any mode change) and provides direct benefit to both SolverSIM and IDA. The remaining `std::map` audit extends the upstream Flat Vector Derivatives work. A4 provides incremental gains by avoiding redundant ordering computations. P6 is excluded from SolverSIM speedup accounting (IDA-only).

**Tasks:**
1. Implement P3 Phase A: structural index map in `ModelManager::evalJtAdept` with conservative invalidation on any mode change.
2. Pre-allocate `SparseMatrix` in `SolverKINEuler::evalJ_KIN` and `SolverIDA::evalJ` to avoid per-call heap allocation.
3. Instrument to measure extraction loop speedup (proportion of entries skipped) on Nordic system and event-heavy cases.
4. Audit remaining `std::map<int, double>` usage outside the Network model's `Derivatives` class (Modelica-generated C++, SubModel interface coupling) and convert where beneficial.
5. Implement A4 (ordering cache with invalidation on structure change).
6. Comprehensive benchmarking of all Phase 1 items combined.
7. Regression testing on the full test suite.

**Gate for P3 Phase B/C:** No Jacobian-structure miss events in the event-heavy regression set (see Structural-Sparsity Validation checklist in Decision Criteria). Phase B (conservative invalidation with instrumentation) and Phase C (severe-only invalidation) proceed only after this gate passes.

**Go/No-Go Criteria for Phase 2:**
- Cumulative speedup (Phase 0 + Phase 1) of at least 20% on the large-scale test case.
- No new test failures.

---

### Phase 2: Advanced

**Objective:** Enable parallel execution and apply build-level optimizations for an additional 15-30% speedup.

**Items:**
- **P7. Profile-Guided Optimization (PGO)** (5-10% speedup) — Build-system-only change with zero code risk.
- **P8. Link-Time Optimization (LTO)** (3-5% speedup) — Straightforward to enable.
- **P1. OpenMP Jacobian Evaluation** (8-12% speedup) — **Known risks from RTE experience.** KLU's internal data structures use global locks that serialize parallel threads during factorization.
- **P2. OpenMP SubModel Evaluation** (3-5% speedup) — **Nested parallelism risk.**

**Rationale:** P7 and P8 are low-risk build-system changes. OpenMP parallelization (P1, P2) has high potential payoff but also high risk. Developer feedback from RTE (Gautier Bureau, TRAISIM discussion) confirms that KLU lock contention is a real problem. The recommendation is to prototype P1 early and measure actual lock contention before committing.

**Tasks:**
1. Set up PGO build infrastructure (P7) with representative workload scripts.
2. Enable LTO (P8) in the build system with an option flag.
3. Benchmark PGO + LTO combined.
4. **P1 feasibility study:** Prototype OpenMP Jacobian evaluation on the Nordic system with 2 and 4 threads and measure KLU lock contention.
5. If viable: thread-safety audit, implementation of P1 and P2.
6. Benchmark OpenMP scaling on 2, 4, 8, and 16 cores.
7. Integration testing of all Phase 2 items combined.

**Go/No-Go Criteria for Phase 3:**
- PGO and LTO builds produce identical numerical results.
- If OpenMP is pursued: provides at least 1.5x speedup on 4 cores.
- If KLU lock contention blocks P1/P2: document findings and evaluate alternatives.
- All numerical results within acceptable tolerance of the sequential baseline.

---

### Phase 3: Research / Exploratory

> **High risk / Long term / Possibly out of scope.** These items require significant architectural changes, carry high implementation risk, and may not be justified unless Phases 0-2 prove insufficient. Each should be evaluated as a research prototype before any production commitment.

**Items:**
- **A5. Partial Jacobian Updates** (10-20% speedup) — High complexity; developer feedback flags cross-model coupling difficulties. May not be needed if Phase 0 factorization avoidance provides sufficient gains.
- **P9. GPU Acceleration for KLU** (20-50% for very large systems) — Heavy dependency (CUDA/cuSOLVER), only beneficial above a large crossover system size.
- **A6. Schur Complement Decomposition** (15-30%) — Requires partitioning network vs. device models, significant architectural change.

**Tasks:**
1. Evaluate whether Phase 0-2 gains are sufficient for the target use cases. If yes, defer Phase 3.
2. If A5 is pursued: prototype with a conservative Newton-failure fallback approach on a small test case before committing.
3. If P9 is pursued: prototype GPU sparse solve using cuSOLVER on extracted Jacobian matrices, measure data transfer overhead.
4. If A6 is pursued: prototype Schur complement solver with manual partitioning.
5. Each prototype must produce a written evaluation report with benchmarks and a go/no-go recommendation.

### Decision Points and Go/No-Go Criteria

> **Speedup accounting note.** Phase speedups are not strictly additive; totals should be measured incrementally with ablation runs. Interactions between optimizations (e.g., reduced factorization frequency changes the relative cost of Jacobian assembly) can shift the balance.

#### Structural-Sparsity Validation (for P3 Phase B/C)

Before promoting P3 from Phase A (structural index map) to Phase B (invalidation instrumentation) or Phase C (severe-only invalidation), the following pass/fail checks must be satisfied:

| Check | Method | Pass Criterion |
|-------|--------|-----------------|
| Pattern hash stability | Compare nnz pattern hash at each Jacobian eval vs. expected structural map | Zero misses across all test cases below |
| Unexpected nonzero counter | Track "nonzero outside structural map" events per simulation | Counter = 0 |
| Nordic baseline | Run Nordic test system (no events, steady-state) | Zero structural map misses |
| Event-heavy case | Run Nordic with tap/OEL/AVR activity (frequent minor events) | Zero structural map misses |
| Severe events | Run Nordic with fault + line disconnection | Zero structural map misses |
| Large-scale case | Run French 6,000+ bus system (if available) | Zero structural map misses |

**Abort rule:** If any miss is detected, abort promotion to Phase C. Investigate whether the miss is caused by a genuine structural change (requiring invalidation) or a numerical artifact. Phase B's conservative invalidation remains the production default until all checks pass.

#### Metrics for All Phases

The following metrics should be tracked across all optimization phases:

| Metric | Measurement Method | Acceptable Threshold |
|--------|-------------------|---------------------|
| Wall-clock speedup | Profiler total time comparison | Must meet minimum target for each phase |
| Numerical accuracy | L2 norm of solution difference vs. baseline | < 1e-6 relative difference |
| Newton iterations | Profiler call count for JacobianEval | No more than 5% increase |
| Time step rejections | Solver retry count | No more than 10% increase |
| Memory usage | Profiler peak RSS | No more than 20% increase |
| Test suite pass rate | CI pipeline | 100% pass (no regressions) |

#### Phase Transition Criteria

**Phase 0 to Phase 1:**
- Minimum 8% measured speedup on the reference large-scale test case.
- All existing tests pass without modification.
- Profiling data confirms factorization skip rate > 50% in steady-state periods.

**Phase 1 to Phase 2:**
- Cumulative speedup of at least 20%.
- No new test failures.

**Phase 2 to Phase 3:**
- Cumulative speedup of at least 40%.
- No numerical accuracy regressions.

**Phase 3 Exit Criteria:**
- Each prototype has a written evaluation report with benchmarks.
- Go/no-go decision documented for each item.
- Productionized items have full test coverage and documentation.

#### Abort Criteria

At any phase, abort and investigate if:
- Any optimization causes a numerical accuracy regression > 1e-4 relative.
- An optimization causes more than 20% increase in Newton iteration failures.
- An optimization causes more than 10% slowdown in any test case (indicating negative interaction).
- Memory usage increases by more than 50%.
- The optimization introduces thread-safety bugs detected by sanitizers (TSan, ASan).
