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
 * @file  DYNTimer.h
 *
 * @brief Class timer header : timers are used to monitored the code execution
 *
 */
#ifndef COMMON_DYNTIMER_H_
#define COMMON_DYNTIMER_H_

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <chrono>
#include <boost/core/noncopyable.hpp>

// #define PRINT_TIMERS

namespace DYN {

/**
 * @brief Phases of the simulation and solver that can be timed
 *
 * Values index fixed-size accumulation arrays, hence the enumeration is
 * contiguous and PHASE_COUNT is last. PHASE_COUNT is also used as the
 * "no parent" sentinel when a phase is entered at the top level.
 */
enum TimerPhase {
  PHASE_SIMULATION_LOOP = 0,
  PHASE_SOLVER_SOLVE,
  PHASE_CALCULATE_IC,
  PHASE_SOLVER_STEP,
  PHASE_JACOBIAN_EVAL,
  PHASE_RESIDUAL_EVAL,
  PHASE_ROOT_EVAL,
  PHASE_MODE_EVAL,
  PHASE_DISCRETE_EVAL,
  PHASE_NR_SOLVE,
  PHASE_MATRIX_COPY,
  PHASE_KINSOL_SOLVE,
  PHASE_REINIT,
  PHASE_IO,
  PHASE_KLU_SYMBOLIC,
  PHASE_KLU_SETUP,
  PHASE_CURVES_UPDATE,
  PHASE_COUNT
};

/**
 * @brief Human readable name of a phase
 * @param phase phase to name
 * @return a static, never null, never empty string
 */
const char* phaseToString(TimerPhase phase);

/**
 * @class Timers
 * @brief Timers clas description : stored all timer created during execution
 *
 */
class Timers : private boost::noncopyable {
 public:
  /**
   * @brief add new statistics for a given timer
   *
   * @param name name of timer
   * @param time time elapsed for the timer
   */
  static void add(const std::string& name, double time);

  /**
   * @brief get the unique instance of Timers in current memory space
   *
   * @return the unique instance
   */
  static Timers& instance();

  /**
   * @brief accumulate one completed phase measurement
   *
   * @param phase phase that completed
   * @param parent phase active when it started, or PHASE_COUNT if none
   * @param time elapsed seconds
   */
  static void record(TimerPhase phase, TimerPhase parent, double time);

  /**
   * @brief clear all phase statistics, used by tests
   */
  static void resetPhases();

  /**
   * @brief mark a phase as entered and report its parent
   *
   * @param phase phase being entered
   * @return the phase that was active, or PHASE_COUNT if none
   */
  static TimerPhase enter(TimerPhase phase);

  /**
   * @brief mark a phase as left
   *
   * @param phase phase being left
   */
  static void exit(TimerPhase phase);

  /**
   * @brief total inclusive time accumulated for a phase
   * @param phase phase to query
   * @return seconds
   */
  double phaseTotal(TimerPhase phase) const;

  /**
   * @brief number of completed measurements for a phase
   * @param phase phase to query
   * @return call count
   */
  uint64_t phaseCalls(TimerPhase phase) const;

  /**
   * @brief shortest single measurement for a phase
   * @param phase phase to query
   * @return milliseconds, or 0 if never measured
   */
  double phaseMinMs(TimerPhase phase) const;

  /**
   * @brief longest single measurement for a phase
   * @param phase phase to query
   * @return milliseconds, or 0 if never measured
   */
  double phaseMaxMs(TimerPhase phase) const;

  /**
   * @brief time spent in a phase excluding all of its children
   * @param phase phase to query
   * @return seconds
   */
  double phaseExclusive(TimerPhase phase) const;

  /**
   * @brief time a child accumulated while nested directly under a parent
   * @param parent parent phase
   * @param child child phase
   * @return seconds
   */
  double phaseParentChild(TimerPhase parent, TimerPhase child) const;

 private:
  /**
   * @brief default constructor, zeroes the phase accumulators
   */
  Timers();

  /**
   * @brief destructor
   */
  ~Timers();

  /**
   * @brief internal add new statistics for a given timer
   *
   * @param name name of timer
   * @param time time elapsed for the timer
   */
  void add_(const std::string& name, double time);

  /**
   * @brief internal accumulate one completed phase measurement
   *
   * @param phase phase that completed
   * @param parent phase active when it started, or PHASE_COUNT if none
   * @param time elapsed seconds
   */
  void record_(TimerPhase phase, TimerPhase parent, double time);

  /**
   * @brief internal clear all phase statistics
   */
  void resetPhases_();

  /**
   * @brief internal mark a phase as entered
   * @param phase phase being entered
   * @return the phase that was active, or PHASE_COUNT if none
   */
  TimerPhase enter_(TimerPhase phase);

  /**
   * @brief internal mark a phase as left
   * @param phase phase being left
   */
  void exit_(TimerPhase phase);

 private:
  std::map<std::string, double> timers_;  ///< association between timers and time elapsed
  std::map<std::string, int32_t> nbAppels_;  ///< association between timers and number of call
  double phaseTime_[PHASE_COUNT];  ///< inclusive seconds accumulated per phase
  uint64_t phaseCalls_[PHASE_COUNT];  ///< completed measurements per phase
  double phaseMin_[PHASE_COUNT];  ///< shortest single measurement per phase, seconds
  double phaseMax_[PHASE_COUNT];  ///< longest single measurement per phase, seconds
  double parentChild_[PHASE_COUNT][PHASE_COUNT];  ///< time a child spent directly under a parent, seconds
  int phaseDepth_[PHASE_COUNT];  ///< current nesting depth per phase, for the recursion rule
  std::vector<TimerPhase> phaseStack_;  ///< phases currently entered, innermost last
};

/**
 * @class Timer
 * @brief Timer clas description :compute time elapsed between its creation
 * and its destruction
 *
 */
class Timer : private boost::noncopyable {
 public:
  /**
   * @brief constructor
   *
   * @param name name of the timer
   */
  explicit Timer(const std::string& name);

  /**
   * @brief destructor
   */
  ~Timer();

  /**
   * @brief stop the timer
   */
  void stop();

  /**
   * @brief Retrieves the elapsed time of the timer
   * @returns the elapsed time of the timer or 0 if the timer is stoppped
   */
  double elapsed() const;

 private:
  std::string name_;  ///< name of timer
  std::chrono::steady_clock::time_point startPoint_;  ///< start time point of the timer
  bool isStopped_;  ///< @b true is the timer is stopped
};

}  // namespace DYN

#endif  // COMMON_DYNTIMER_H_
