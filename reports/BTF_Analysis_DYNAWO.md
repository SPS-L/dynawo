# BTF (Block Triangular Form) Analysis in DYNAWO Power System Simulation

**Author:** Sustainable Power Systems Lab (SPSL)  
**Website:** https://sps-lab.org  
**Contact:** info@sps-lab.org  
**Date:** September 18, 2025

## Overview

This document provides a comprehensive analysis of the BTF (Block Triangular Form) library usage in DYNAWO, specifically focusing on the `btf_l_maxtrans` function and its role in the power system simulation solver chain.

## Table of Contents

1. [Introduction to BTF](#introduction-to-btf)
2. [The `btf_l_maxtrans` Function](#the-btf_l_maxtrans-function)
3. [DYNAWO Solver Architecture](#dynawo-solver-architecture)
4. [The Complete Call Chain](#the-complete-call-chain)
5. [When BTF Functions Are Called](#when-btf-functions-are-called)
6. [Performance Optimization Strategy](#performance-optimization-strategy)
7. [Code Analysis](#code-analysis)
8. [Conclusion](#conclusion)

## Introduction to BTF

The **BTF (Block Triangular Form)** library is part of the SuiteSparse collection of sparse matrix algorithms. In DYNAWO, BTF is used internally by the **KLU sparse linear solver** to optimize the solution of large sparse linear systems that arise during power system simulation.

### Key Components:
- **SuiteSparse**: Collection of sparse matrix libraries
- **KLU**: Sparse LU factorization algorithm
- **BTF**: Block triangular form preprocessing
- **KINSOL**: Newton-Raphson nonlinear solver (SUNDIALS)

## The `btf_l_maxtrans` Function

### Purpose
The `btf_l_maxtrans` function implements a **maximum transversal algorithm** that:

1. **Finds optimal column permutations**: Creates a permutation matrix `Q` such that `A×Q` has a zero-free diagonal
2. **Detects structural singularity**: Determines if the matrix can be factorized
3. **Maximizes diagonal elements**: Ensures the best possible pivot selection for numerical stability

### Algorithm Details
- **Input**: Sparse matrix structure (sparsity pattern)
- **Output**: Column permutation vector and matching information
- **Result**: If successful, every row is matched to a column with a non-zero entry
- **Failure case**: If structurally singular, some `Match[i]` entries are set to -1

### Mathematical Context
For a sparse matrix `A`, the function finds permutation `Q` such that:
- `A×Q` has non-zero diagonal elements
- The permuted matrix is suitable for stable LU factorization
- Fill-in during factorization is minimized

## DYNAWO Solver Architecture

### Hierarchical Structure

```
Power System Simulation (DYNAWO)
├── SimulationRT::simulate()
├── Fixed Time Step Solvers
│   ├── SolverSIM (Simplified/Backward Euler)
│   └── SolverTRAP (Trapezoidal)
├── Variable Time Step Solver
│   └── SolverIDA (BDF methods)
└── Algebraic Solvers
    ├── SolverKINEuler (Newton-Raphson)
    └── SolverKINAlgRestoration (Mode restoration)
```

### Linear Algebra Stack

```
DYNAWO Model (DAE System)
├── Jacobian Matrix Construction
├── KINSOL (Newton-Raphson)
├── KLU Linear Solver
├── BTF Preprocessing ← btf_l_maxtrans called here
└── Sparse LU Factorization
```

## The Complete Call Chain

### 1. High-Level Simulation Flow

```cpp
// SimulationRT.cpp - Main simulation loop
while (!end() && !SignalHandler::gotExitSignal() && criteriaChecked) {
    solver_->solve(tStop_, tCurrent_);  // Entry point
}
```

### 2. Fixed Time Step Solver

```cpp
// DYNSolverCommonFixedTimeStep.cpp
void SolverCommonFixedTimeStep::solveStepCommon(double tAim, double& tNxt) {
    do {
        handleMaximumTries(counter);
        h_ = hNew_;
        
        // Critical path - algebraic solver call
        if (model_->sizeY() != 0) {
            int flag = callAlgebraicSolver();  // ← Key function
            status = analyzeResult(flag);
            if (status != NON_CONV)
                updateZAndMode(status);
        }
        
        // Handle convergence/divergence
        switch (status) {
            case NON_CONV: handleDivergence(redoStep); break;
            case CONV: handleConvergence(redoStep); break;
            case ROOT: handleRoot(redoStep); break;
        }
    } while (redoStep);
}
```

### 3. Algebraic Solver Interface

```cpp
// DYNSolverCommonFixedTimeStep.cpp
int SolverCommonFixedTimeStep::callAlgebraicSolver() {
    if (skipNextNR_) {
        return KIN_INITIAL_GUESS_OK;
    } else {
        computePrediction();
        
        // Factorization control logic
        bool noInitSetup = true;
        if (stats_.nst_ == 0 || factorizationForced_)
            noInitSetup = false;  // ← Forces BTF preprocessing
        
        // Newton-Raphson solve
        flag = solverKINEuler_->solve(noInitSetup, skipAlgebraicResidualsEvaluation_);
        updateStatistics();
    }
    return flag;
}
```

### 4. KINSOL Integration

```cpp
// DYNSolverKINCommon.cpp - KLU linear solver setup
linearSolver_ = SUNLinSol_KLU(sundialsVectorY_, sundialsMatrix_, sundialsContext_);
flag = KINSetLinearSolver(KINMem_, linearSolver_, sundialsMatrix_);
```

### 5. BTF Preprocessing (Internal to KLU)

When `noInitSetup = false`, KLU performs:
1. **Symbolic Analysis** (including `btf_l_maxtrans`)
2. **Numerical Factorization**

## When BTF Functions Are Called

### Automatic Triggering Conditions

The `btf_l_maxtrans` function is called automatically when:

#### 1. **Initial Setup** (`noInitSetup = false`)
- **First time step**: `stats_.nst_ == 0`
- **After convergence failure**: `factorizationForced_ = true`
- **After mode changes**: Requiring Jacobian updates

#### 2. **Convergence Failure Recovery**
```cpp
void SolverCommonFixedTimeStep::handleDivergence(bool& redoStep) {
    if (doubleEquals(h_, hMin_)) {
        throw DYNError(Error::SOLVER_ALGO, SolverFixedTimeStepConvFailMin, solverType());
    }
    factorizationForced_ = true;  // ← Forces BTF preprocessing
    redoStep = true;
    decreaseStep();
    restoreContinuousVariables();
}
```

#### 3. **Algebraic Mode Changes**
```cpp
void SolverCommonFixedTimeStep::handleRoot(bool& redoStep) {
    if (model_->getModeChangeType() == ALGEBRAIC_J_UPDATE_MODE) {
        factorizationForced_ = true;  // ← Forces BTF preprocessing
    } else {
        factorizationForced_ = false;
        increaseStep();
    }
    redoStep = false;
}
```

### Optimization for Efficiency
```cpp
void SolverCommonFixedTimeStep::handleConvergence(bool& redoStep) {
    factorizationForced_ = false;  // ← Avoids BTF preprocessing
    redoStep = false;
    skipAlgebraicResidualsEvaluation_ = optimizeAlgebraicResidualsEvaluations_;
    increaseStep();
}
```

## Performance Optimization Strategy

### 1. **Factorization Reuse Strategy**
- **Expensive operations**: Symbolic factorization (including BTF)
- **Reuse when possible**: `noInitSetup = true` skips preprocessing
- **Force when necessary**: Critical for convergence and stability

### 2. **Adaptive Time Stepping**
```cpp
// Increase step size after successful convergence
void SolverCommonFixedTimeStep::increaseStep() {
    if (doubleNotEquals(h_, hMax_))
        hNew_ = min(h_ / kReduceStep_, hMax_);
    hNew_ = min(hNew_, tEnd_ - (tSolve_ + h_));
}

// Decrease step size after divergence
void SolverCommonFixedTimeStep::decreaseStep() {
    hNew_ = max(h_ * kReduceStep_, hMin_);
}
```

### 3. **Smart Jacobian Updates**
The solver intelligently decides when to update the Jacobian matrix:
- **Avoid updates**: When convergence is good (`factorizationForced_ = false`)
- **Force updates**: After failures or discrete changes (`factorizationForced_ = true`)

## Code Analysis

### Key Files and Functions

#### 1. **Main Solver Loop**
- **File**: `dynawort/dynawo/sources/Simulation/DYNSimulationRT.cpp`
- **Function**: `SimulationRT::simulate()`
- **Line**: ~240: `solver_->solve(tStop_, tCurrent_);`

#### 2. **Fixed Time Step Common Logic**
- **File**: `dynawort/dynawo/sources/Solvers/FixedTimeStep/DYNSolverCommonFixedTimeStep.cpp`
- **Key Functions**:
  - `solveStepCommon()` (line 177)
  - `callAlgebraicSolver()` (line 308)
  - `handleDivergence()` (line 363)
  - `handleConvergence()` (line 379)

#### 3. **KLU Integration**
- **File**: `dynawort/dynawo/sources/Solvers/AlgebraicSolvers/DYNSolverKINCommon.cpp`
- **Line**: ~140: `linearSolver_ = SUNLinSol_KLU(...)`

#### 4. **BTF Library Integration**
- **File**: `dynawort/dynawo/3rdParty/suitesparse/patch/common/suitesparse.patch`
- **Source**: `Source/btf_maxtrans.c` (part of SuiteSparse)

### Factorization Control Logic Flow

```cpp
// Step 1: Determine if factorization is needed
bool noInitSetup = true;
if (stats_.nst_ == 0 || factorizationForced_)
    noInitSetup = false;

// Step 2: Call solver with factorization flag
flag = solverKINEuler_->solve(noInitSetup, skipAlgebraicResidualsEvaluation_);

// Step 3: Handle result and set future factorization policy
switch (status) {
    case NON_CONV:
        factorizationForced_ = true;   // Force next factorization
        break;
    case CONV:
        factorizationForced_ = false;  // Avoid next factorization
        break;
    case ROOT:
        // Conditional based on mode change type
        factorizationForced_ = (model_->getModeChangeType() == ALGEBRAIC_J_UPDATE_MODE);
        break;
}
```

### Complete Call Stack Summary

```
SimulationRT::simulate() 
  → solver_->solve()
    → SolverCommonFixedTimeStep::solveStepCommon()
      → callAlgebraicSolver()
        → solverKINEuler_->solve(noInitSetup, ...)
          → KINSOL Newton-Raphson iterations
            → KLU linear solver (when Jacobian needs factorization)
              → BTF preprocessing (including btf_l_maxtrans)
                → Actual LU factorization
                → Linear system solution
```

## Conclusion

### Summary of BTF Role in DYNAWO

1. **Critical Preprocessing**: `btf_l_maxtrans` ensures optimal matrix structure for LU factorization
2. **Automatic Operation**: Called internally by KLU, transparent to DYNAWO application code
3. **Performance Impact**: Essential for efficient sparse matrix operations in large power systems
4. **Intelligent Control**: DYNAWO's solver architecture minimizes expensive factorizations while ensuring numerical stability

### Key Insights

- **BTF preprocessing** is the most computationally expensive part of linear system solving
- **DYNAWO's adaptive strategy** balances performance and robustness
- **Matrix structure optimization** is crucial for large-scale power system simulations
- **The solver hierarchy** efficiently manages when expensive operations are necessary

### Technical Significance

The `btf_l_maxtrans` function represents a critical component in the computational chain that enables DYNAWO to simulate large, complex power systems efficiently. By ensuring optimal matrix structure before factorization, it directly contributes to:

- **Numerical stability** of the simulation
- **Computational efficiency** of linear solvers  
- **Robustness** of the overall simulation process
- **Scalability** to large power system models

This analysis demonstrates the sophisticated engineering behind modern power system simulation tools, where multiple layers of mathematical algorithms work together to solve complex differential-algebraic equation systems representing electrical networks.

### Research Applications

This understanding is particularly valuable for:
- **Power system researchers** developing new simulation methods
- **Software developers** optimizing large-scale numerical simulations
- **Engineers** troubleshooting convergence issues in power system studies
- **Students** learning about the computational aspects of power system analysis

The BTF library's role exemplifies how fundamental linear algebra research directly enables practical engineering applications in critical infrastructure simulation.
