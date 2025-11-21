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
 * @file  DYNSolverProfiler.cpp
 *
 * @brief Performance monitoring and profiling utilities implementation
 *
 * Author: Sustainable Power Systems Lab (SPS-L)
 * Website: https://sps-lab.org
 * Contact: info@sps-lab.org
 */

#include "DYNSolverProfiler.h"
#include "DYNTrace.h"

#include <iomanip>
#include <sstream>

namespace DYN {

SolverProfiler::SolverProfiler() :
symbolicFactorizations_(0),
numericalOnlyFactorizations_(0),
structureChangeDetections_(0),
falsePositiveStructureChanges_(0),
jacobianEvaluations_(0),
totalSymbolicTime_(0.0),
totalNumericalTime_(0.0),
totalJacobianTime_(0.0),
totalNnzDiff_(0.0),
totalChangeRatio_(0.0) {
}

SolverProfiler::~SolverProfiler() {
}

void SolverProfiler::reset() {
  symbolicFactorizations_ = 0;
  numericalOnlyFactorizations_ = 0;
  structureChangeDetections_ = 0;
  falsePositiveStructureChanges_ = 0;
  jacobianEvaluations_ = 0;
  totalSymbolicTime_ = 0.0;
  totalNumericalTime_ = 0.0;
  totalJacobianTime_ = 0.0;
  totalNnzDiff_ = 0.0;
  totalChangeRatio_ = 0.0;
}

void SolverProfiler::logSymbolicFactorization(double elapsedTime) {
  symbolicFactorizations_++;
  totalSymbolicTime_ += elapsedTime;
}

void SolverProfiler::logNumericalFactorization(double elapsedTime) {
  numericalOnlyFactorizations_++;
  totalNumericalTime_ += elapsedTime;
}

void SolverProfiler::logStructureChange(bool wasNecessary, int nnzDiff, double changeRatio) {
  structureChangeDetections_++;
  totalNnzDiff_ += nnzDiff;
  totalChangeRatio_ += changeRatio;
  
  if (!wasNecessary) {
    falsePositiveStructureChanges_++;
  }
}

void SolverProfiler::logJacobianEvaluation(double elapsedTime) {
  jacobianEvaluations_++;
  totalJacobianTime_ += elapsedTime;
}

double SolverProfiler::getSymbolicToNumericalRatio() const {
  if (numericalOnlyFactorizations_ == 0) {
    return 0.0;
  }
  return static_cast<double>(symbolicFactorizations_) / numericalOnlyFactorizations_;
}

void SolverProfiler::printStatistics() const {
  std::stringstream ss;
  
  ss << "\n";
  ss << "========================================\n";
  ss << "  Solver Performance Profiling Report  \n";
  ss << "========================================\n";
  ss << "\n";
  
  // Factorization statistics
  ss << "Factorization Statistics:\n";
  ss << "  Symbolic factorizations:        " << symbolicFactorizations_ << "\n";
  ss << "  Numerical-only factorizations:  " << numericalOnlyFactorizations_ << "\n";
  
  int totalFactorizations = symbolicFactorizations_ + numericalOnlyFactorizations_;
  if (totalFactorizations > 0) {
    double symbolicRatio = static_cast<double>(symbolicFactorizations_) / totalFactorizations * 100.0;
    ss << "  Symbolic ratio:                 " << std::fixed << std::setprecision(1) 
       << symbolicRatio << "%\n";
  }
  
  if (symbolicFactorizations_ > 0) {
    double avgSymbolicTime = totalSymbolicTime_ / symbolicFactorizations_;
    ss << "  Avg symbolic time:              " << std::fixed << std::setprecision(6) 
       << avgSymbolicTime << " s\n";
  }
  
  if (numericalOnlyFactorizations_ > 0) {
    double avgNumericalTime = totalNumericalTime_ / numericalOnlyFactorizations_;
    ss << "  Avg numerical time:             " << std::fixed << std::setprecision(6) 
       << avgNumericalTime << " s\n";
  }
  
  ss << "  Total symbolic time:            " << std::fixed << std::setprecision(6) 
     << totalSymbolicTime_ << " s\n";
  ss << "  Total numerical time:           " << std::fixed << std::setprecision(6) 
     << totalNumericalTime_ << " s\n";
  
  double ratio = getSymbolicToNumericalRatio();
  if (ratio > 0.0) {
    ss << "  Symbolic/Numerical ratio:       " << std::fixed << std::setprecision(2) 
       << ratio << ":1\n";
  }
  
  ss << "\n";
  
  // Matrix structure change statistics
  ss << "Matrix Structure Changes:\n";
  ss << "  Total detections:               " << structureChangeDetections_ << "\n";
  ss << "  False positives avoided:        " << falsePositiveStructureChanges_ << "\n";
  
  if (structureChangeDetections_ > 0) {
    double avoidanceRate = static_cast<double>(falsePositiveStructureChanges_) / 
                           structureChangeDetections_ * 100.0;
    ss << "  Avoidance rate:                 " << std::fixed << std::setprecision(1) 
       << avoidanceRate << "%\n";
    
    double avgNnzDiff = totalNnzDiff_ / structureChangeDetections_;
    ss << "  Avg NNZ difference:             " << std::fixed << std::setprecision(1) 
       << avgNnzDiff << "\n";
    
    double avgChangeRatio = totalChangeRatio_ / structureChangeDetections_;
    ss << "  Avg change ratio:               " << std::fixed << std::setprecision(4) 
       << avgChangeRatio << "\n";
  }
  
  ss << "\n";
  
  // Jacobian evaluation statistics
  ss << "Jacobian Evaluation:\n";
  ss << "  Total evaluations:              " << jacobianEvaluations_ << "\n";
  ss << "  Total Jacobian time:            " << std::fixed << std::setprecision(6) 
     << totalJacobianTime_ << " s\n";
  
  if (jacobianEvaluations_ > 0) {
    double avgJacobianTime = totalJacobianTime_ / jacobianEvaluations_;
    ss << "  Avg evaluation time:            " << std::fixed << std::setprecision(6) 
       << avgJacobianTime << " s\n";
  }
  
  ss << "\n";
  
  // Performance insights
  ss << "Performance Analysis:\n";
  
  if (ratio > 1.5) {
    ss << "  ⚠ HIGH symbolic factorization ratio detected!\n";
    ss << "    Consider enabling adaptive factorization control.\n";
  } else if (ratio > 0.0 && ratio < 0.5) {
    ss << "  ✓ GOOD symbolic factorization efficiency.\n";
  }
  
  if (falsePositiveStructureChanges_ > 0) {
    double savedTime = falsePositiveStructureChanges_ * 
                       (totalSymbolicTime_ / symbolicFactorizations_);
    ss << "  Estimated time saved by tolerance: " << std::fixed << std::setprecision(3) 
       << savedTime << " s\n";
  }
  
  ss << "========================================\n";
  
  Trace::info() << ss.str() << Trace::endline;
}

// ScopedTimer implementation
ScopedTimer::ScopedTimer() :
startTime_(std::chrono::high_resolution_clock::now()) {
}

ScopedTimer::~ScopedTimer() {
}

double ScopedTimer::getElapsedSeconds() const {
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = endTime - startTime_;
  return elapsed.count();
}

}  // namespace DYN

