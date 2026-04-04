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
 * @brief Solver performance profiling framework
 *
 * Provides per-phase timing, memory tracking, call counters,
 * and CSV/JSON export for solver performance analysis.
 * Enabled at build time via the DYNAWO_PROFILING CMake option.
 * In non-profiling builds all macros expand to no-ops.
 */

#ifndef SOLVERS_COMMON_DYNSOLVERPROFILER_H_
#define SOLVERS_COMMON_DYNSOLVERPROFILER_H_

#include <string>
#include <vector>
#include <chrono>
#include <cstdint>

namespace DYN {

/**
 * @brief Enumeration of solver phases that can be profiled
 */
enum ProfilePhase {
  PHASE_SIMULATION_LOOP = 0,
  PHASE_SOLVER_SOLVE,
  PHASE_CALCULATE_IC,
  PHASE_SOLVER_STEP,
  PHASE_JACOBIAN_EVAL,
  PHASE_RESIDUAL_EVAL,
  PHASE_ROOT_EVAL,
  PHASE_MODE_EVAL,
  PHASE_DISCRETE_EVAL,
  PHASE_NR_SOLVE,      ///< Newton-Raphson algebraic solve (fixed-timestep solvers)
  PHASE_MATRIX_COPY,
  PHASE_KINSOL_SOLVE,
  PHASE_REINIT,
  PHASE_IO,
  PHASE_KLU_SYMBOLIC,  ///< KLU symbolic factorization (klu_analyze), triggered on sparsity-structure change
  PHASE_KLU_SETUP,     ///< SUNLinSolSetup (klu_refactor / klu_factor), intercepted via per-instance ops-patching
  PHASE_CURVES_UPDATE,  ///< Mid-simulation updateCurves() calls (curve buffer flush + calculated-var eval)
  PHASE_COUNT  ///< Total number of phases (must be last)
};

/**
 * @brief Get a human-readable name for a profiling phase
 * @param phase the phase enum value
 * @return string name of the phase
 */
const char* phaseToString(ProfilePhase phase);

/**
 * @brief Statistics for a single profiled phase
 */
struct PhaseStats {
  double totalTime;         ///< Total accumulated time in seconds
  double minTime;           ///< Minimum single-call time in seconds
  double maxTime;           ///< Maximum single-call time in seconds
  uint64_t callCount;       ///< Number of times this phase was entered
  uint64_t peakMemoryKB;    ///< Peak RSS observed during this phase (KB)

  PhaseStats();
  void reset();
  double avgTime() const;
};

/**
 * @brief Singleton solver profiler collecting per-phase performance data
 *
 * Not thread-safe; designed for single-threaded solver execution.
 * Activated at build time via the DYNAWO_PROFILING CMake option.
 */
class SolverProfiler {
 public:
  /**
   * @brief Get the singleton profiler instance
   * @return reference to the global profiler
   */
  static SolverProfiler& instance();

  /**
   * @brief Check if profiling is currently enabled
   * @return true if profiling is active
   */
  bool isEnabled() const;

  /**
   * @brief Enable or disable profiling at runtime
   * @param enabled true to enable, false to disable
   */
  void setEnabled(bool enabled);

  /**
   * @brief Reset all collected statistics
   */
  void reset();

  /**
   * @brief Record a timing measurement for a phase
   * @param phase the phase that was measured
   * @param elapsedSeconds time elapsed in seconds
   */
  void record(ProfilePhase phase, double elapsedSeconds);

  /**
   * @brief Record a timing measurement for a phase under a specific parent context
   * @param phase the phase that was measured
   * @param parentPhase parent phase active when this phase started, or PHASE_COUNT if none
   * @param elapsedSeconds time elapsed in seconds
   */
  void recordWithParent(ProfilePhase phase, ProfilePhase parentPhase, double elapsedSeconds);

  /**
   * @brief Record a timing measurement with memory snapshot
   * @param phase the phase that was measured
   * @param elapsedSeconds time elapsed in seconds
   * @param memoryKB current RSS in kilobytes
   */
  void recordWithMemory(ProfilePhase phase, double elapsedSeconds, uint64_t memoryKB);

  /**
   * @brief Record a timing measurement with memory snapshot and parent context
   * @param phase the phase that was measured
   * @param parentPhase parent phase active when this phase started, or PHASE_COUNT if none
   * @param elapsedSeconds time elapsed in seconds
   * @param memoryKB current RSS in kilobytes
   */
  void recordWithMemoryAndParent(ProfilePhase phase, ProfilePhase parentPhase, double elapsedSeconds, uint64_t memoryKB);

  /**
   * @brief Notify profiler that a phase scope was entered
   * @param phase entered phase
   */
  void enterPhase(ProfilePhase phase);

  /**
   * @brief Notify profiler that a phase scope was exited
   * @param phase exited phase
   */
  void exitPhase(ProfilePhase phase);

  /**
   * @brief Get current active phase from stack
   * @return active phase or PHASE_COUNT if stack is empty
   */
  ProfilePhase currentPhase() const;

  /**
   * @brief Record a timestep entry for time-series analysis
   * @param simTime simulation time
   * @param stepDurationMs wall-clock duration of the step in milliseconds
   * @param memoryKB current RSS in kilobytes
   */
  void recordTimestep(double simTime, double stepDurationMs, uint64_t memoryKB);

  /**
   * @brief Get statistics for a specific phase
   * @param phase the phase to query
   * @return const reference to the phase statistics
   */
  const PhaseStats& getStats(ProfilePhase phase) const;

  /**
   * @brief Get total profiled wall-clock time across all phases
   * @return total time in seconds
   */
  double totalTime() const;

  /**
   * @brief Print a summary report to the Dynawo trace log
   */
  void printReport() const;

  /**
   * @brief Export profiling data to CSV file
   * @param filename path to the output CSV file
   */
  void exportCSV(const std::string& filename) const;

  /**
   * @brief Export profiling data to JSON file
   * @param filename path to the output JSON file
   */
  void exportJSON(const std::string& filename) const;

  /**
   * @brief Get current process RSS memory from /proc/self/status
   * @return RSS in kilobytes, or 0 if unavailable
   */
  static uint64_t getCurrentMemoryKB();

 private:
  SolverProfiler();
  ~SolverProfiler();

  // Non-copyable
  SolverProfiler(const SolverProfiler&);
  SolverProfiler& operator=(const SolverProfiler&);

  bool enabled_;
  PhaseStats stats_[PHASE_COUNT];

  /**
   * @brief Per-timestep record for time-series export
   */
  struct TimestepRecord {
    double simTime;
    double stepDurationMs;
    uint64_t memoryKB;
  };
  std::vector<TimestepRecord> timestepRecords_;
  double parentChildTime_[PHASE_COUNT][PHASE_COUNT];
  std::vector<ProfilePhase> phaseStack_;
};

/**
 * @brief RAII timer that automatically records elapsed time for a phase
 *
 * Create on the stack at the beginning of a profiled scope.
 * The destructor records the elapsed time to the global SolverProfiler.
 */
class PhaseTimer {
 public:
  /**
   * @brief Construct and start timing
   * @param phase the phase being timed
   * @param trackMemory if true, also record memory at end of phase
   */
  explicit PhaseTimer(ProfilePhase phase, bool trackMemory = false);

  /**
   * @brief Stop timing and record to the profiler
   */
  ~PhaseTimer();

  /**
   * @brief Get elapsed time so far (without stopping)
   * @return elapsed seconds
   */
  double elapsed() const;

 private:
  ProfilePhase phase_;
  ProfilePhase parentPhase_;
  bool trackMemory_;
  bool active_;
  std::chrono::high_resolution_clock::time_point start_;
};

}  // namespace DYN

// Profiling macros - defined as no-ops when DYNAWO_PROFILING is not set
#ifdef DYNAWO_PROFILING
  #define DYN_PROFILE_PHASE(phase) \
    DYN::PhaseTimer dynProfileTimer_##phase(DYN::phase)
  #define DYN_PROFILE_PHASE_MEM(phase) \
    DYN::PhaseTimer dynProfileTimer_##phase(DYN::phase, true)
  #define DYN_PROFILE_RECORD_TIMESTEP(simTime, stepMs, memKB) \
    DYN::SolverProfiler::instance().recordTimestep(simTime, stepMs, memKB)
  #define DYN_PROFILE_PRINT_REPORT() \
    DYN::SolverProfiler::instance().printReport()
  #define DYN_PROFILE_EXPORT_CSV(filename) \
    DYN::SolverProfiler::instance().exportCSV(filename)
  #define DYN_PROFILE_EXPORT_JSON(filename) \
    DYN::SolverProfiler::instance().exportJSON(filename)
#else
  #define DYN_PROFILE_PHASE(phase) ((void)0)
  #define DYN_PROFILE_PHASE_MEM(phase) ((void)0)
  #define DYN_PROFILE_RECORD_TIMESTEP(simTime, stepMs, memKB) ((void)0)
  #define DYN_PROFILE_PRINT_REPORT() ((void)0)
  #define DYN_PROFILE_EXPORT_CSV(filename) ((void)0)
  #define DYN_PROFILE_EXPORT_JSON(filename) ((void)0)
#endif

#endif  // SOLVERS_COMMON_DYNSOLVERPROFILER_H_
