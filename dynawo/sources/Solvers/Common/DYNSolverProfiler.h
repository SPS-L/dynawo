//
// Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source time domain
// simulation tool for power systems.
//

/**
 * @file  DYNSolverProfiler.h
 *
 * @brief Performance monitoring and profiling utilities for solver optimization
 *
 * This class provides comprehensive performance tracking for Jacobian factorization
 * and matrix operations to help identify optimization opportunities and validate
 * performance improvements.
 *
 * Author: Sustainable Power Systems Lab (SPS-L)
 * Website: https://sps-lab.org
 * Contact: info@sps-lab.org
 */
#ifndef SOLVERS_COMMON_DYNSOLVERPROFILE_H_
#define SOLVERS_COMMON_DYNSOLVERPROFILE_H_

#include <chrono>
#include <string>

namespace DYN {

/**
 * @brief Performance profiler for solver operations
 *
 * Tracks factorization statistics, matrix structure changes, and timing information
 * to help optimize solver performance and validate optimization strategies.
 */
class SolverProfiler {
 public:
  /**
   * @brief Constructor
   */
  SolverProfiler();

  /**
   * @brief Destructor
   */
  ~SolverProfiler();

  /**
   * @brief Reset all statistics to zero
   */
  void reset();

  /**
   * @brief Log a symbolic factorization event
   * @param elapsedTime time taken for the symbolic factorization in seconds
   */
  void logSymbolicFactorization(double elapsedTime);

  /**
   * @brief Log a numerical-only factorization event
   * @param elapsedTime time taken for the numerical factorization in seconds
   */
  void logNumericalFactorization(double elapsedTime);

  /**
   * @brief Log a matrix structure change detection
   * @param wasNecessary true if the structure change was significant and necessary
   * @param nnzDiff absolute difference in number of non-zeros
   * @param changeRatio relative change ratio (0.0 to 1.0)
   */
  void logStructureChange(bool wasNecessary, int nnzDiff, double changeRatio);

  /**
   * @brief Log a Jacobian evaluation
   * @param elapsedTime time taken for Jacobian evaluation in seconds
   */
  void logJacobianEvaluation(double elapsedTime);

  /**
   * @brief Print comprehensive statistics to trace output
   */
  void printStatistics() const;

  /**
   * @brief Get total number of symbolic factorizations performed
   * @return count of symbolic factorizations
   */
  int getSymbolicFactorizationCount() const { return symbolicFactorizations_; }

  /**
   * @brief Get total number of numerical factorizations performed
   * @return count of numerical factorizations
   */
  int getNumericalFactorizationCount() const { return numericalOnlyFactorizations_; }

  /**
   * @brief Get total symbolic factorization time
   * @return total time in seconds
   */
  double getTotalSymbolicTime() const { return totalSymbolicTime_; }

  /**
   * @brief Get total numerical factorization time
   * @return total time in seconds
   */
  double getTotalNumericalTime() const { return totalNumericalTime_; }

  /**
   * @brief Get symbolic to numerical factorization ratio
   * @return ratio (higher values indicate excessive symbolic factorizations)
   */
  double getSymbolicToNumericalRatio() const;

  /**
   * @brief Get number of false positive structure changes
   * @return count of unnecessary structure change detections
   */
  int getFalsePositiveCount() const { return falsePositiveStructureChanges_; }

 private:
  int symbolicFactorizations_;           ///< Count of symbolic factorizations
  int numericalOnlyFactorizations_;      ///< Count of numerical-only factorizations
  int structureChangeDetections_;        ///< Total structure change detections
  int falsePositiveStructureChanges_;    ///< Structure changes avoided by tolerance
  int jacobianEvaluations_;              ///< Count of Jacobian evaluations

  double totalSymbolicTime_;             ///< Total time in symbolic factorization (seconds)
  double totalNumericalTime_;            ///< Total time in numerical factorization (seconds)
  double totalJacobianTime_;             ///< Total time in Jacobian evaluation (seconds)

  double totalNnzDiff_;                  ///< Cumulative NNZ differences
  double totalChangeRatio_;              ///< Cumulative change ratios
};

/**
 * @brief RAII timer for measuring operation duration
 *
 * Automatically records elapsed time when object goes out of scope.
 */
class ScopedTimer {
 public:
  /**
   * @brief Constructor - starts the timer
   */
  ScopedTimer();

  /**
   * @brief Destructor - stops the timer
   */
  ~ScopedTimer();

  /**
   * @brief Get elapsed time since construction
   * @return elapsed time in seconds
   */
  double getElapsedSeconds() const;

 private:
  std::chrono::high_resolution_clock::time_point startTime_;
};

}  // end of namespace DYN

#endif  // SOLVERS_COMMON_DYNSOLVERPROFILE_H_

