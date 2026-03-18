# Dynawo Optimization Roadmap

This document presents a detailed optimization plan for the Dynawo power system simulation tool. It covers 10 programming optimizations and 10 algorithmic optimizations, each with descriptions, expected speedup ranges, implementation effort and risk assessments, and implementation sketches. A phased roadmap at the end provides a structured plan for executing these improvements.

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
   - [P1. Flat Vector Derivatives](#p1-flat-vector-derivatives)
   - [P2. OpenMP Jacobian Evaluation](#p2-openmp-jacobian-evaluation)
   - [P3. OpenMP SubModel Evaluation](#p3-openmp-submodel-evaluation)
   - [P4. Cache-Optimized Sparse Matrix Layout](#p4-cache-optimized-sparse-matrix-layout)
   - [P5. SIMD Vectorization in Residual Evaluation](#p5-simd-vectorization-in-residual-evaluation)
   - [P6. Memory Pool Allocator](#p6-memory-pool-allocator)
   - [P7. Reduce SUNDIALS N_Vector Copies](#p7-reduce-sundials-nvector-copies)
   - [P8. Profile-Guided Optimization (PGO)](#p8-profile-guided-optimization-pgo)
   - [P9. Link-Time Optimization (LTO)](#p9-link-time-optimization-lto)
   - [P10. GPU Acceleration for KLU](#p10-gpu-acceleration-for-klu)
2. [Algorithmic Optimizations](#algorithmic-optimizations)
   - [A1. Adaptive Factorization Control](#a1-adaptive-factorization-control)
   - [A2. Matrix Structure Change Tolerance](#a2-matrix-structure-change-tolerance)
   - [A3. KLU Numerical-Only Refactorization](#a3-klu-numerical-only-refactorization)
   - [A4. Improved COLAMD Ordering](#a4-improved-colamd-ordering)
   - [A5. Partial Jacobian Updates](#a5-partial-jacobian-updates)
   - [A6. Adaptive Time Step Control](#a6-adaptive-time-step-control)
   - [A7. Improved Newton Convergence Criteria](#a7-improved-newton-convergence-criteria)
   - [A8. Krylov Preconditioner Strategies](#a8-krylov-preconditioner-strategies)
   - [A9. Schur Complement Decomposition](#a9-schur-complement-decomposition)
   - [A10. Waveform Relaxation for Multi-Rate Simulation](#a10-waveform-relaxation-for-multi-rate-simulation)
3. [Phased Roadmap](#phased-roadmap)
   - [Phase 0: Quick Wins (1-2 weeks)](#phase-0-quick-wins-1-2-weeks)
   - [Phase 1: Medium Effort (2-4 weeks)](#phase-1-medium-effort-2-4-weeks)
   - [Phase 2: Advanced (1-3 months)](#phase-2-advanced-1-3-months)
   - [Phase 3: Research (3-6 months)](#phase-3-research-3-6-months)
   - [Decision Points and Go/No-Go Criteria](#decision-points-and-gono-go-criteria)

---

## Programming Optimizations

### P1. Flat Vector Derivatives

**Expected Speedup:** 8-10%
**Implementation Effort:** Medium
**Risk Level:** Low

#### Description

The current `Derivatives` class and related data structures in Dynawo use `std::map<int, double>` to store sparse derivative values indexed by variable ID. While `std::map` provides convenient insertion and lookup for arbitrary keys, its red-black tree implementation incurs significant overhead: each entry requires a separate heap allocation for the tree node (typically 48-64 bytes per entry on 64-bit systems, compared to 8 bytes for the actual `double` value), and traversal follows pointer chains that are hostile to CPU cache prefetching.

For the Jacobian evaluation and residual computation phases, which are called thousands of times per simulation and dominate the runtime, replacing `std::map<int, double>` with flat `std::vector<double>` indexed by dense variable IDs can eliminate this overhead. Since variable indices in a given submodel are typically dense and known at initialization time, a flat vector with direct indexing is both feasible and natural.

The primary challenge is handling the sparse-to-dense transition at model boundaries, where variable indices may not be contiguous. This can be addressed with an index remapping table computed once during initialization, converting global sparse indices to dense local indices.

#### Implementation Sketch

```cpp
// Before: in Derivatives class
class Derivatives {
  std::map<int, double> values_;  // sparse, heap-heavy
public:
  void setValue(int varIdx, double val) { values_[varIdx] = val; }
  double getValue(int varIdx) const {
    auto it = values_.find(varIdx);
    return (it != values_.end()) ? it->second : 0.0;
  }
};

// After: flat vector with index remapping
class Derivatives {
  std::vector<double> values_;         // dense, cache-friendly
  std::vector<int> globalToLocal_;     // index remapping table
  int localSize_;

public:
  void init(const std::vector<int>& globalIndices) {
    localSize_ = static_cast<int>(globalIndices.size());
    values_.resize(localSize_, 0.0);
    int maxGlobal = *std::max_element(globalIndices.begin(), globalIndices.end());
    globalToLocal_.assign(maxGlobal + 1, -1);
    for (int i = 0; i < localSize_; ++i) {
      globalToLocal_[globalIndices[i]] = i;
    }
  }

  void setValue(int varIdx, double val) {
    int local = globalToLocal_[varIdx];
    if (local >= 0)
      values_[local] = val;
  }

  double getValue(int varIdx) const {
    int local = globalToLocal_[varIdx];
    return (local >= 0) ? values_[local] : 0.0;
  }

  void clear() {
    std::fill(values_.begin(), values_.end(), 0.0);
  }

  // Direct access for inner loops (no bounds check)
  double* data() { return values_.data(); }
  int size() const { return localSize_; }
};
```

---

### P2. OpenMP Jacobian Evaluation

**Expected Speedup:** 8-12%
**Implementation Effort:** Medium
**Risk Level:** Medium

#### Description

The Jacobian evaluation phase (`ModelMulti::evalJt()`) iterates over all submodels and evaluates each one's contribution to the global Jacobian matrix. In the current implementation, this loop is sequential: each submodel's Jacobian is computed one after another. Since submodels are largely independent during the evaluation phase (they read shared state but write to non-overlapping regions of the Jacobian matrix), this loop is amenable to OpenMP parallelization.

The main complication is that each submodel writes to a different row/column block of the sparse Jacobian. As long as the write regions do not overlap -- which is guaranteed by the block structure of the multi-model Jacobian -- the loop can be parallelized without locks. Care must be taken with thread-local scratch buffers and ensuring that no submodel modifies shared state during Jacobian evaluation.

The expected speedup depends on the number of submodels and their relative evaluation cost. For simulations with many generator models (each with its own Jacobian block), the speedup scales well with core count up to the number of submodels. For network-dominated simulations with few large submodels, the speedup is more modest.

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

### P3. OpenMP SubModel Evaluation

**Expected Speedup:** 3-5%
**Implementation Effort:** Medium
**Risk Level:** Medium

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

### P4. Cache-Optimized Sparse Matrix Layout

**Expected Speedup:** 2-3%
**Implementation Effort:** Medium
**Risk Level:** Low

#### Description

Sparse matrices in Dynawo are stored for use with the KLU direct solver from SuiteSparse. KLU operates on Compressed Sparse Column (CSC) format, which stores non-zero entries column by column. However, many operations during Jacobian assembly naturally produce data in row-major order, leading to an intermediate representation that must be transposed before KLU can use it.

By aligning the internal Jacobian assembly to directly produce CSC format -- or by using Compressed Sparse Row (CSR) format where row-major access patterns dominate and converting to CSC only at the KLU interface -- we can reduce cache misses during both assembly and solve phases. The key insight is that Jacobian assembly typically fills the matrix row by row (each equation contributes a row), while KLU needs column-by-column storage.

A detailed analysis of access patterns in the current code is needed to determine the optimal layout. For medium-sized systems (1000-5000 equations), the assembly phase may benefit from CSR storage with a single transpose, while for larger systems, direct CSC assembly with column-oriented accumulation buffers may be better.

#### Implementation Sketch

```cpp
// Direct CSC assembly with per-column accumulation
class DirectCSCAssembler {
  // Column pointers, row indices, values (standard CSC)
  std::vector<int> colPtr_;
  std::vector<int> rowIdx_;
  std::vector<double> values_;

  // Per-column temporary lists for assembly
  struct ColEntry {
    int row;
    double val;
  };
  std::vector<std::vector<ColEntry>> colAccum_;

public:
  void beginAssembly(int nCols) {
    colAccum_.resize(nCols);
    for (auto& col : colAccum_) col.clear();
  }

  // Called during Jacobian evaluation (can be from any row order)
  void addEntry(int row, int col, double val) {
    colAccum_[col].push_back({row, val});
  }

  // Convert accumulated entries to CSC format for KLU
  void finalize() {
    int nCols = static_cast<int>(colAccum_.size());
    colPtr_.resize(nCols + 1);
    rowIdx_.clear();
    values_.clear();

    int nnz = 0;
    for (int j = 0; j < nCols; ++j) {
      colPtr_[j] = nnz;
      // Sort by row for cache-friendly access during solve
      std::sort(colAccum_[j].begin(), colAccum_[j].end(),
                [](const ColEntry& a, const ColEntry& b) {
                  return a.row < b.row;
                });
      for (const auto& e : colAccum_[j]) {
        rowIdx_.push_back(e.row);
        values_.push_back(e.val);
      }
      nnz += static_cast<int>(colAccum_[j].size());
    }
    colPtr_[nCols] = nnz;
  }

  // Provide CSC data to KLU
  int* colPtrData() { return colPtr_.data(); }
  int* rowIdxData() { return rowIdx_.data(); }
  double* valuesData() { return values_.data(); }
};
```

---

### P5. SIMD Vectorization in Residual Evaluation

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

### P6. Memory Pool Allocator

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

### P7. Reduce SUNDIALS N_Vector Copies

**Expected Speedup:** 1-3%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

The SUNDIALS IDA solver interface uses `N_Vector` objects to pass state vectors, derivative vectors, and residual vectors between the solver and user code. In the current Dynawo-SUNDIALS integration, there are several places where vector data is copied between `N_Vector` internal storage and Dynawo's own `std::vector<double>` containers. These copies occur at every solver call (residual evaluation, Jacobian evaluation), adding up to significant overhead for large systems.

Many of these copies can be eliminated by wrapping Dynawo's existing vectors as `N_Vector`s using SUNDIALS' custom vector operations interface, or by directly using `NV_DATA_S()` to access the `N_Vector` data pointer and passing it to Dynawo functions without intermediate copies. The SUNDIALS serial `N_Vector` stores its data as a contiguous `double*` array, so direct pointer access is safe and efficient.

This optimization has the lowest risk of any proposed change because it only affects the SUNDIALS interface layer, not the solver logic or model evaluation code. The main caution is ensuring that the `N_Vector` data is not reallocated or freed while Dynawo holds a pointer to it.

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

### P8. Profile-Guided Optimization (PGO)

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

### P9. Link-Time Optimization (LTO)

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

### P10. GPU Acceleration for KLU

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
**Risk Level:** Medium

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

**Expected Speedup:** 10-20%
**Implementation Effort:** High
**Risk Level:** Medium

#### Description

In a multi-model simulation, the global Jacobian matrix is block-structured: each submodel contributes a diagonal block, and the network model provides coupling blocks. When only a subset of submodels have changed state significantly (e.g., a generator hitting a limit while others are in steady state), only the corresponding diagonal blocks need to be recomputed. The unchanged blocks can be reused from the previous evaluation.

Implementing partial Jacobian updates requires: (1) A change-detection mechanism that identifies which submodels have experienced significant state changes since the last Jacobian evaluation. This can be based on the norm of the state variable change vector for each submodel, compared to a threshold. (2) A Jacobian assembly framework that can selectively update blocks while preserving the rest of the matrix. (3) A refactorization strategy that accounts for the partially-updated matrix (typically, numerical refactorization is sufficient if only values changed, not structure).

The potential speedup is large because Jacobian evaluation is typically the most expensive phase, and in many simulations only a small fraction of submodels are actively changing at any given time. However, the implementation is complex and carries medium risk because an incorrectly reused Jacobian block can lead to Newton convergence failures or incorrect results.

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

### A6. Adaptive Time Step Control

**Expected Speedup:** 3-10%
**Implementation Effort:** Medium
**Risk Level:** Medium

#### Description

The IDA solver in SUNDIALS uses an adaptive time step controller based on local truncation error estimates. However, the default controller parameters may not be optimal for power system dynamics, which exhibit long periods of quasi-steady-state behavior punctuated by fast transients (faults, switching events). A more aggressive time step strategy -- larger steps during steady-state periods and smaller steps only near events -- can reduce the total number of time steps significantly.

Three improvements are proposed: (1) Increase the maximum time step size during quasi-steady-state periods, detected by monitoring the rate of change of key state variables (rotor angles, voltage magnitudes). (2) Implement predictive step size reduction before known events (events with scheduled times can be anticipated). (3) Use a variable-order strategy where the BDF order is also adapted based on the solution smoothness, allowing higher-order (and thus larger time step) methods during smooth phases.

The risk is medium because overly aggressive time stepping can miss fast dynamics or cause solver convergence failures. The implementation must include fallback behavior (automatic step reduction) when convergence fails, which IDA already provides. The tuning of the aggressiveness parameters should be validated on a range of test cases.

#### Implementation Sketch

```cpp
class AdaptiveStepController {
  double maxStepSteadyState_;   // max dt during quasi-steady state
  double maxStepTransient_;     // max dt during transients
  double steadyStateThreshold_; // threshold for quasi-steady detection

  // Sliding window of state change rates
  std::deque<double> recentChangeRates_;
  int windowSize_;

public:
  AdaptiveStepController()
      : maxStepSteadyState_(0.1),
        maxStepTransient_(0.001),
        steadyStateThreshold_(1e-3),
        windowSize_(10) {}

  double computeMaxStep(double t, const double* y, const double* yp,
                        int n, double nextEventTime) {

    // 1. Compute current state change rate
    double changeRate = 0.0;
    for (int i = 0; i < n; ++i) {
      changeRate += yp[i] * yp[i];
    }
    changeRate = std::sqrt(changeRate / n);

    recentChangeRates_.push_back(changeRate);
    if (static_cast<int>(recentChangeRates_.size()) > windowSize_) {
      recentChangeRates_.pop_front();
    }

    // 2. Determine if system is in quasi-steady state
    double avgRate = 0.0;
    for (double r : recentChangeRates_) avgRate += r;
    avgRate /= recentChangeRates_.size();

    bool isSteadyState = (avgRate < steadyStateThreshold_);

    // 3. Set max step based on state
    double maxStep = isSteadyState ? maxStepSteadyState_ : maxStepTransient_;

    // 4. Reduce step if approaching a known event
    if (nextEventTime > t) {
      double timeToEvent = nextEventTime - t;
      if (timeToEvent < 2.0 * maxStep) {
        // Approach the event gradually
        maxStep = std::min(maxStep, timeToEvent / 3.0);
      }
    }

    return maxStep;
  }
};

// Integration with IDA:
// Before each IDA step, call IDASetMaxStep() with the computed max step
void SolverIDA::step(double tNext) {
  double maxDt = stepController_.computeMaxStep(
      tCurrent_, yData_, ypData_, N_, nextEventTime_);
  IDASetMaxStep(idaMem_, maxDt);
  int flag = IDASolve(idaMem_, tNext, &tReturn_, yVec_, ypVec_, IDA_NORMAL);
  // ... handle flag ...
}
```

---

### A7. Improved Newton Convergence Criteria

**Expected Speedup:** 2-5%
**Implementation Effort:** Low
**Risk Level:** Low

#### Description

The Newton solver used within IDA (and KINSOL for initial conditions) uses convergence criteria based on the weighted RMS norm of the correction vector. The default convergence test may be either too strict (causing unnecessary extra iterations) or too loose (allowing poor solutions that cause time step failures). Tuning these criteria for power system simulations can reduce the average number of Newton iterations per time step.

For power systems, a natural improvement is to use component-weighted norms that account for the different scales of state variables. Rotor angles (radians), voltage magnitudes (per-unit), and generator speeds (per-unit) have different natural scales, and a single uniform tolerance may over-solve some components while under-solving others. By providing SUNDIALS with appropriate error weight functions that normalize each component by its expected range, the Newton solver converges more efficiently.

Additionally, the convergence rate of the Newton iterations can be monitored to detect early when the iteration is unlikely to converge, allowing a faster fallback to a smaller time step. If the correction norm is not decreasing at a sufficient rate after 2-3 iterations, it is better to reduce the time step immediately rather than exhausting the maximum iteration count.

#### Implementation Sketch

```cpp
// Custom error weight function for IDA
int customErrWeight(N_Vector y, N_Vector ewt, void* userData) {
  SolverData* data = static_cast<SolverData*>(userData);
  double* yData = NV_DATA_S(y);
  double* ewtData = NV_DATA_S(ewt);
  int n = NV_LENGTH_S(y);

  for (int i = 0; i < n; ++i) {
    double scale;
    switch (data->varTypes[i]) {
      case VAR_ANGLE:
        scale = 1.0;          // radians, O(1)
        break;
      case VAR_VOLTAGE:
        scale = 1.0;          // per-unit, O(1)
        break;
      case VAR_SPEED:
        scale = 1.0 / 377.0;  // rad/s to per-unit
        break;
      case VAR_CURRENT:
        scale = 0.1;          // currents can be larger
        break;
      default:
        scale = 1.0;
    }
    double atol = data->absTol * scale;
    double rtol = data->relTol;
    double denom = rtol * std::abs(yData[i]) + atol;
    ewtData[i] = (denom > 0) ? 1.0 / denom : 1.0 / atol;
  }
  return 0;
}

// Early convergence failure detection
class NewtonConvergenceMonitor {
  double lastCorrNorm_;
  int iterCount_;
  double minReductionRate_;   // correction must shrink by this factor per iter

public:
  NewtonConvergenceMonitor() : lastCorrNorm_(0), iterCount_(0),
                                minReductionRate_(0.5) {}

  void reset() {
    lastCorrNorm_ = 0;
    iterCount_ = 0;
  }

  // Returns true if convergence looks unlikely
  bool shouldAbort(double corrNorm) {
    ++iterCount_;
    if (iterCount_ >= 3 && lastCorrNorm_ > 0) {
      double reductionRate = corrNorm / lastCorrNorm_;
      if (reductionRate > minReductionRate_) {
        // Convergence rate too slow; abort and reduce step
        return true;
      }
    }
    lastCorrNorm_ = corrNorm;
    return false;
  }
};

// Register the custom weight function with IDA:
// IDAWFtolerances(idaMem_, customErrWeight);
```

---

### A8. Krylov Preconditioner Strategies

**Expected Speedup:** 20-40% (for very large systems, 5000+ equations)
**Implementation Effort:** High
**Risk Level:** Medium

#### Description

For very large power system models (5000+ equations), the direct sparse LU solver (KLU) becomes the dominant bottleneck because the factorization cost grows superlinearly with matrix size. Krylov iterative methods (GMRES, BiCGStab) scale more favorably but require effective preconditioners to converge in a reasonable number of iterations.

Two preconditioner strategies are well-suited to power system Jacobians: (1) Incomplete LU factorization (ILU) computes an approximate LU factorization with controlled fill-in, providing a good trade-off between setup cost and convergence acceleration. ILU(0) uses the same sparsity pattern as the original matrix and is cheap to compute; ILU(k) allows k levels of fill for better accuracy. (2) Block-Jacobi preconditioning exploits the block structure of the multi-model Jacobian: each submodel's diagonal block is factored exactly (using KLU), while the coupling blocks are approximated or ignored. This is highly parallel and works well when the coupling between submodels is relatively weak.

The crossover point where Krylov methods outperform direct solvers depends on the system size and conditioning. For well-conditioned systems below 5000 equations, KLU is typically faster. For larger systems or poorly conditioned matrices, Krylov with a good preconditioner can be 2-5x faster. SUNDIALS provides built-in support for Krylov linear solvers (SUNLINSOL_SPGMR, SUNLINSOL_SPBCGS) that integrate with IDA.

#### Implementation Sketch

```cpp
// Block-Jacobi preconditioner using KLU for diagonal blocks
class BlockJacobiPreconditioner {
  struct Block {
    klu_symbolic* symbolic;
    klu_numeric* numeric;
    klu_common common;
    int size;
    int offset;
    std::vector<int> Ap, Ai;
    std::vector<double> Ax;
  };

  std::vector<Block> blocks_;

public:
  void setup(const std::vector<SubModel*>& subModels,
             const SparseMatrix& fullJacobian,
             const std::vector<int>& offsets,
             const std::vector<int>& sizes) {

    blocks_.resize(subModels.size());

    for (size_t i = 0; i < subModels.size(); ++i) {
      Block& b = blocks_[i];
      b.size = sizes[i];
      b.offset = offsets[i];
      klu_defaults(&b.common);

      // Extract diagonal block from full Jacobian
      extractDiagonalBlock(fullJacobian, offsets[i], sizes[i],
                           b.Ap, b.Ai, b.Ax);

      // Factor diagonal block
      b.symbolic = klu_analyze(b.size, b.Ap.data(), b.Ai.data(), &b.common);
      b.numeric = klu_factor(b.Ap.data(), b.Ai.data(), b.Ax.data(),
                             b.symbolic, &b.common);
    }
  }

  // Apply preconditioner: solve M*z = r (approximately)
  int apply(const double* r, double* z, int n) {
    // Solve each diagonal block independently
    #pragma omp parallel for schedule(dynamic) if(blocks_.size() > 4)
    for (size_t i = 0; i < blocks_.size(); ++i) {
      Block& b = blocks_[i];
      // Copy r[offset:offset+size] to z[offset:offset+size]
      std::copy(r + b.offset, r + b.offset + b.size, z + b.offset);
      // Solve block system
      klu_solve(b.symbolic, b.numeric, b.size, 1,
                z + b.offset, &b.common);
    }
    return 0;
  }

  ~BlockJacobiPreconditioner() {
    for (auto& b : blocks_) {
      if (b.numeric) klu_free_numeric(&b.numeric, &b.common);
      if (b.symbolic) klu_free_symbolic(&b.symbolic, &b.common);
    }
  }

private:
  void extractDiagonalBlock(const SparseMatrix& J, int offset, int size,
                            std::vector<int>& Ap, std::vector<int>& Ai,
                            std::vector<double>& Ax) {
    // Extract the submatrix J[offset:offset+size, offset:offset+size]
    Ap.resize(size + 1);
    Ai.clear();
    Ax.clear();
    Ap[0] = 0;
    for (int col = 0; col < size; ++col) {
      int gCol = col + offset;
      for (int idx = J.colPtr(gCol); idx < J.colPtr(gCol + 1); ++idx) {
        int gRow = J.rowIdx(idx);
        if (gRow >= offset && gRow < offset + size) {
          Ai.push_back(gRow - offset);
          Ax.push_back(J.value(idx));
        }
      }
      Ap[col + 1] = static_cast<int>(Ai.size());
    }
  }
};
```

---

### A9. Schur Complement Decomposition

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

### A10. Waveform Relaxation for Multi-Rate Simulation

**Expected Speedup:** 10-25% (for systems with widely varying time constants)
**Implementation Effort:** High
**Risk Level:** High

#### Description

Power system simulations often contain subsystems with very different time scales: fast electromagnetic transients in power electronics (microsecond scale), electromechanical dynamics in generators (millisecond to second scale), and slow controls and protection systems (second to minute scale). A monolithic solver must use a time step small enough for the fastest dynamics, which wastes computation on the slow subsystems.

Waveform relaxation (WR) decomposes the system into subsystems that can be integrated independently with different time step sizes, then iterates to account for the coupling between subsystems. Each subsystem uses a time step appropriate to its own dynamics: fast subsystems take many small steps while slow subsystems take fewer large steps. The coupling is handled by exchanging waveforms (interpolated solution trajectories) between subsystems at synchronization points.

For a power system with N_fast fast variables and N_slow slow variables, WR can reduce the total number of function evaluations from O((N_fast + N_slow) / dt_fast) to O(N_fast / dt_fast + N_slow / dt_slow + K * N_coupling) where K is the number of WR iterations and N_coupling is the coupling dimension. If the coupling is weak (as is typical between electromagnetic and electromechanical dynamics), K is small (2-5 iterations).

The risk is high because WR introduces an outer iteration loop whose convergence depends on the coupling strength and the synchronization interval. Poorly chosen partitioning or synchronization intervals can lead to divergence. Extensive testing on representative systems is required, and a fallback to monolithic simulation should be available when WR convergence is poor.

#### Implementation Sketch

```cpp
class WaveformRelaxationSolver {
  struct Subsystem {
    std::unique_ptr<SubSystemSolver> solver;
    std::vector<int> varIndices;      // global variable indices
    double dtMax;                     // maximum time step for this subsystem
    std::vector<double> waveform;     // interpolated coupling input
    int rate;                         // relative time step ratio
  };

  std::vector<Subsystem> subsystems_;
  double syncInterval_;               // WR synchronization interval
  int maxWRIterations_;
  double wrTolerance_;

public:
  WaveformRelaxationSolver(double syncInterval = 0.01,
                           int maxIter = 5,
                           double tol = 1e-4)
      : syncInterval_(syncInterval),
        maxWRIterations_(maxIter),
        wrTolerance_(tol) {}

  void addSubsystem(std::unique_ptr<SubSystemSolver> solver,
                    const std::vector<int>& varIdx,
                    double dtMax, int rate) {
    subsystems_.push_back({std::move(solver), varIdx, dtMax, {}, rate});
  }

  // Advance from tStart to tStart + syncInterval
  int solveWindow(double tStart, double* yGlobal) {
    double tEnd = tStart + syncInterval_;

    // Store previous waveforms for convergence check
    std::vector<std::vector<double>> prevWaveforms(subsystems_.size());

    for (int wrIter = 0; wrIter < maxWRIterations_; ++wrIter) {
      // Integrate each subsystem independently
      for (size_t i = 0; i < subsystems_.size(); ++i) {
        Subsystem& sub = subsystems_[i];

        // Save previous waveform
        prevWaveforms[i] = sub.waveform;

        // Extract local state from global vector
        std::vector<double> yLocal(sub.varIndices.size());
        for (size_t j = 0; j < sub.varIndices.size(); ++j) {
          yLocal[j] = yGlobal[sub.varIndices[j]];
        }

        // Integrate from tStart to tEnd with subsystem's own time step
        sub.solver->integrate(tStart, tEnd, yLocal.data(),
                              sub.waveform, sub.dtMax);

        // Write back to global vector
        for (size_t j = 0; j < sub.varIndices.size(); ++j) {
          yGlobal[sub.varIndices[j]] = yLocal[j];
        }
      }

      // Exchange waveforms between subsystems
      exchangeWaveforms();

      // Check convergence
      double maxChange = 0.0;
      for (size_t i = 0; i < subsystems_.size(); ++i) {
        if (!prevWaveforms[i].empty()) {
          for (size_t j = 0; j < subsystems_[i].waveform.size(); ++j) {
            double diff = std::abs(subsystems_[i].waveform[j] - prevWaveforms[i][j]);
            maxChange = std::max(maxChange, diff);
          }
        }
      }

      if (maxChange < wrTolerance_) {
        return 0;  // Converged
      }
    }

    return -1;  // Did not converge; fall back to monolithic
  }

private:
  void exchangeWaveforms() {
    // For each subsystem, update its coupling input waveform
    // from the latest solutions of the other subsystems.
    // This involves interpolation when subsystems have different
    // time step sizes.
    for (size_t i = 0; i < subsystems_.size(); ++i) {
      subsystems_[i].solver->updateCouplingInputs(subsystems_);
    }
  }
};
```

---

## Phased Roadmap

### Phase 0: Quick Wins (1-2 weeks)

**Objective:** Achieve 10-18% overall speedup with minimal code changes and low risk.

**Items:**
- **A1. Adaptive Factorization Control** (5-8% speedup)
- **A2. Matrix Structure Change Tolerance** (3-5% speedup)
- **A3. KLU Numerical-Only Refactorization** (5-7% speedup)

**Rationale:** These three algorithmic optimizations all target the sparse linear algebra pipeline, which typically consumes 40-60% of simulation time. They are complementary: A1 provides the decision logic, A2 extends the criteria for when to skip symbolic factorization, and A3 ensures the fast refactorization path is used. Together they form a layered factorization strategy that can be implemented and tested incrementally.

**Tasks:**
1. Instrument the current factorization code to measure how often symbolic vs. numerical factorization is performed (1 day).
2. Implement A1 (adaptive factorization controller with structure hash) (2 days).
3. Implement A3 (explicit `klu_refactor` path) alongside A1 (1 day).
4. Implement A2 (structure change tolerance) with configurable threshold (2 days).
5. Benchmark on IEEE 14, IEEE 39, and a large-scale test case (1 day).
6. Tune tolerance parameters based on benchmarks (1 day).
7. Run the full test suite to verify correctness (1 day).

**Go/No-Go Criteria for Phase 1:**
- At least 8% measured speedup on the large-scale test case.
- No increase in Newton iteration failures or convergence warnings.
- All unit tests and integration tests pass.

---

### Phase 1: Medium Effort (2-4 weeks)

**Objective:** Achieve an additional 15-30% speedup through a combination of data structure improvements, build optimizations, and partial Jacobian updates.

**Items:**
- **P1. Flat Vector Derivatives** (8-10% speedup)
- **P8. Profile-Guided Optimization (PGO)** (5-10% speedup)
- **P9. Link-Time Optimization (LTO)** (3-5% speedup)
- **A5. Partial Jacobian Updates** (10-20% speedup)

**Rationale:** P1 addresses a fundamental data structure inefficiency that affects every Jacobian and residual evaluation. P8 and P9 are build-system-only changes that improve all code paths simultaneously. A5 is the algorithmic optimization with the highest payoff that does not require changing the solver architecture.

**Tasks:**
1. Implement P1: flat vector derivatives with index remapping (3-4 days).
2. Audit all `std::map<int, double>` usage in derivatives and Jacobian assembly (2 days).
3. Benchmark P1 in isolation to confirm the 8-10% estimate (1 day).
4. Set up PGO build infrastructure (P8) with representative workload scripts (2 days).
5. Enable LTO (P9) in the build system with an option flag (1 day).
6. Benchmark PGO + LTO combined (1 day).
7. Implement A5: partial Jacobian update with change detection (4-5 days).
8. Implement convergence fallback for A5 (when partial update leads to Newton failure, re-evaluate full Jacobian) (2 days).
9. Comprehensive benchmarking of all Phase 1 items combined (2 days).
10. Regression testing on the full test suite (2 days).

**Go/No-Go Criteria for Phase 2:**
- Cumulative speedup (Phase 0 + Phase 1) of at least 25% on the large-scale test case.
- PGO and LTO builds produce identical numerical results (bitwise or within tolerance).
- Partial Jacobian updates do not increase the total number of Newton iterations by more than 5%.
- No new test failures.

---

### Phase 2: Advanced (1-3 months)

**Objective:** Enable parallel execution and improve solver adaptivity for an additional 15-30% speedup, with architecture changes that enable further scaling.

**Items:**
- **P2. OpenMP Jacobian Evaluation** (8-12% speedup)
- **P3. OpenMP SubModel Evaluation** (3-5% speedup)
- **A6. Adaptive Time Step Control** (3-10% speedup)
- **A7. Improved Newton Convergence Criteria** (2-5% speedup)
- **A8. Krylov Preconditioner Strategies** (20-40% for large systems)

**Rationale:** OpenMP parallelization (P2, P3) requires a thread-safety audit of all submodel code, which is the main time investment. Adaptive time stepping (A6) and Newton criteria (A7) improve the solver's efficiency without changing the underlying algorithms. The Krylov preconditioner (A8) targets very large systems that may emerge as use cases grow.

**Tasks:**
1. Thread-safety audit of SubModel interface and implementations (1-2 weeks).
2. Add thread-safety annotations and fix any shared mutable state (1 week).
3. Implement P2: OpenMP Jacobian evaluation with dynamic scheduling (3-4 days).
4. Implement P3: OpenMP residual and root evaluation (2-3 days).
5. Benchmark OpenMP scaling on 2, 4, 8, and 16 cores (2 days).
6. Implement A7: custom error weight function and early convergence detection (3-4 days).
7. Implement A6: adaptive time step controller with quasi-steady-state detection (1 week).
8. Validate A6 on event-heavy scenarios (fault ride-through, load shedding) (1 week).
9. Implement A8: block-Jacobi preconditioner with SUNLINSOL_SPGMR integration (2 weeks).
10. Benchmark A8 on large-scale systems (5000+ equations) to find crossover point (1 week).
11. Integration testing of all Phase 2 items combined (1 week).

**Go/No-Go Criteria for Phase 3:**
- OpenMP provides at least 1.5x speedup on 4 cores for the standard benchmark suite.
- Krylov solver with preconditioner is faster than KLU for systems above the determined crossover size.
- Adaptive time stepping reduces total step count by at least 15% on quasi-steady-state test cases.
- All numerical results are within acceptable tolerance of the sequential baseline.

---

### Phase 3: Research (3-6 months)

**Objective:** Explore advanced techniques that can provide 2-5x speedup for the largest and most complex simulations, requiring significant architectural changes.

**Items:**
- **P10. GPU Acceleration for KLU** (20-50% for very large systems)
- **A9. Schur Complement Decomposition** (15-30%)
- **A10. Waveform Relaxation for Multi-Rate Simulation** (10-25%)

**Rationale:** These items require substantial development effort and carry higher risk, but offer the largest potential speedups. They are research-oriented and should be evaluated as prototypes before committing to production integration. GPU acceleration (P10) and Schur complement (A9) both target the linear algebra bottleneck from different angles; waveform relaxation (A10) targets the time-stepping bottleneck for multi-rate systems.

**Tasks:**
1. Prototype P10: GPU sparse solve using cuSOLVER on extracted Jacobian matrices from benchmark runs (2-3 weeks).
2. Measure data transfer overhead and determine crossover system size for GPU advantage (1 week).
3. If GPU crossover is favorable, integrate GPU solve path with runtime selection (2-3 weeks).
4. Prototype A9: Schur complement solver with manual partitioning for a specific test case (2-3 weeks).
5. Implement automatic partitioning based on model type (network vs. device) (2 weeks).
6. Benchmark A9 against full KLU and Krylov approaches on multiple system sizes (1 week).
7. Prototype A10: waveform relaxation with manual partitioning into fast/slow subsystems (3-4 weeks).
8. Implement convergence monitoring and fallback to monolithic solver (1 week).
9. Benchmark A10 on multi-rate test cases (power electronics + electromechanical) (2 weeks).
10. Evaluate which Phase 3 items to productionize based on prototype results (1 week).

**Decision Points:**
- After P10 prototype: decide if GPU acceleration is worth the dependency complexity.
- After A9 prototype: decide if Schur complement provides enough benefit over Krylov preconditioners.
- After A10 prototype: decide if waveform relaxation is robust enough for production use.

---

### Decision Points and Go/No-Go Criteria

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
- Cumulative speedup of at least 25%.
- PGO/LTO build pipeline is automated and reproducible.
- Partial Jacobian update fallback mechanism has been exercised and validated.

**Phase 2 to Phase 3:**
- Cumulative speedup of at least 40%.
- OpenMP scaling efficiency > 60% on 4 cores.
- Krylov solver performance crossover point has been empirically determined.
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
