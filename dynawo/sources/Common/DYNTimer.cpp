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
 * @file  DYNTimer.cpp
 *
 * @brief  Class timer implementation : timers are used to monitored the code execution
 *
 */
#include <iostream>

#include "DYNTimer.h"

#include "DYNTrace.h"

#include <limits>
#include <thread>
#include <sstream>
#include <string>

namespace DYN {

const char*
phaseToString(const TimerPhase phase) {
  switch (phase) {
    case PHASE_SIMULATION_LOOP: return "SimulationLoop";
    case PHASE_SOLVER_SOLVE:    return "SolverSolve";
    case PHASE_CALCULATE_IC:    return "CalculateIC";
    case PHASE_SOLVER_STEP:     return "SolverStep";
    case PHASE_JACOBIAN_EVAL:   return "JacobianEval";
    case PHASE_RESIDUAL_EVAL:   return "ResidualEval";
    case PHASE_ROOT_EVAL:       return "RootEval";
    case PHASE_MODE_EVAL:       return "ModeEval";
    case PHASE_DISCRETE_EVAL:   return "DiscreteEval";
    case PHASE_NR_SOLVE:        return "NRSolve";
    case PHASE_MATRIX_COPY:     return "MatrixCopy";
    case PHASE_KINSOL_SOLVE:    return "KINSOLSolve";
    case PHASE_REINIT:          return "Reinit";
    case PHASE_IO:              return "IO";
    case PHASE_KLU_SYMBOLIC:    return "KLUSymbolic";
    case PHASE_KLU_SETUP:       return "KLUSetup";
    case PHASE_CURVES_UPDATE:   return "CurvesUpdate";
    case PHASE_COUNT:           return "None";
  }
  return "Unknown";
}

Timers::Timers() {
  resetPhases_();
}

Timers::~Timers() {
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  for (const auto& timer : timers_)
    std::cout << "TIMER[" << timer.first << "] = " << timer.second << " seconds in " << nbAppels_[timer.first] << " calls" << std::endl;
#endif
}

Timers&
Timers::instance() {
  static thread_local Timers instance;
  return instance;
}

void
Timers::add(const std::string& name, const double time) {
  Timers& timers = instance();
  timers.add_(name, time);
}

void
Timers::add_(const std::string& name, const double time) {
  std::stringstream ss;
  ss << std::this_thread::get_id() << "_" << name;
  std::string name_formatted = ss.str();
  timers_[name_formatted] += time;
  nbAppels_[name_formatted] += 1;
}

void
Timers::resetPhases() {
  instance().resetPhases_();
}

void
Timers::resetPhases_() {
  for (int i = 0; i < PHASE_COUNT; ++i) {
    phaseTime_[i] = 0.;
    phaseCalls_[i] = 0;
    phaseMin_[i] = std::numeric_limits<double>::max();
    phaseMax_[i] = 0.;
    phaseDepth_[i] = 0;
    for (int j = 0; j < PHASE_COUNT; ++j)
      parentChild_[i][j] = 0.;
  }
  phaseStack_.clear();
  cycleWarned_ = false;
}

TimerPhase
Timers::enter(const TimerPhase phase) {
  return instance().enter_(phase);
}

TimerPhase
Timers::enter_(const TimerPhase phase) {
  const TimerPhase parent = phaseStack_.empty() ? PHASE_COUNT : phaseStack_.back();
  // Indirect recursion (A -> B -> A): phase is already open somewhere on the
  // stack but is not the current top. Not supported, see enter()'s Doxygen;
  // warn once per instance rather than assert, since the build that runs
  // cross-validation is not necessarily a debug build.
  if (!cycleWarned_ && phaseDepth_[phase] > 0 && !phaseStack_.empty() && phaseStack_.back() != phase) {
    Trace::warn() << "DYNTimer: indirect recursion detected on phase " << phaseToString(phase)
                  << ", its exclusive time attribution is no longer reliable" << Trace::endline;
    cycleWarned_ = true;
  }
  phaseStack_.push_back(phase);
  phaseDepth_[phase] += 1;
  return parent;
}

void
Timers::exit(const TimerPhase phase) {
  instance().exit_(phase);
}

void
Timers::exit_(const TimerPhase phase) {
  if (!phaseStack_.empty())
    phaseStack_.pop_back();
  if (phaseDepth_[phase] > 0)
    phaseDepth_[phase] -= 1;
}

void
Timers::record(const TimerPhase phase, const TimerPhase parent, const double time) {
  instance().record_(phase, parent, time);
}

void
Timers::record_(const TimerPhase phase, const TimerPhase parent, const double time) {
  // Rule 2: an inner instance of a phase already on the stack records nothing,
  // its time is already contained in the outermost instance's total.
  if (phaseDepth_[phase] > 1)
    return;

  phaseTime_[phase] += time;
  phaseCalls_[phase] += 1;
  if (time < phaseMin_[phase])
    phaseMin_[phase] = time;
  if (time > phaseMax_[phase])
    phaseMax_[phase] = time;

  // Rule 3: no edge at the top level, and never a self edge.
  if (parent != PHASE_COUNT && parent != phase)
    parentChild_[parent][phase] += time;
}

double
Timers::phaseTotal(const TimerPhase phase) const {
  if (phase >= PHASE_COUNT)
    return 0.;
  return phaseTime_[phase];
}

uint64_t
Timers::phaseCalls(const TimerPhase phase) const {
  if (phase >= PHASE_COUNT)
    return 0;
  return phaseCalls_[phase];
}

double
Timers::phaseMinMs(const TimerPhase phase) const {
  if (phase >= PHASE_COUNT)
    return 0.;
  return (phaseCalls_[phase] == 0) ? 0. : phaseMin_[phase] * 1000.;
}

double
Timers::phaseMaxMs(const TimerPhase phase) const {
  if (phase >= PHASE_COUNT)
    return 0.;
  return (phaseCalls_[phase] == 0) ? 0. : phaseMax_[phase] * 1000.;
}

double
Timers::phaseExclusive(const TimerPhase phase) const {
  if (phase >= PHASE_COUNT)
    return 0.;
  double exclusive = phaseTime_[phase];
  for (int c = 0; c < PHASE_COUNT; ++c)
    exclusive -= parentChild_[phase][c];
  return exclusive;
}

double
Timers::phaseParentChild(const TimerPhase parent, const TimerPhase child) const {
  if (parent >= PHASE_COUNT || child >= PHASE_COUNT)
    return 0.;
  return parentChild_[parent][child];
}

bool
Timers::cycleDetected() const {
  return cycleWarned_;
}

Timer::Timer(const std::string& name) :
name_(name),
startPoint_(std::chrono::steady_clock::now()),
isStopped_(false),
phase_(PHASE_COUNT),
parentPhase_(PHASE_COUNT) {
}

Timer::Timer(const TimerPhase phase) :
name_(),
startPoint_(std::chrono::steady_clock::now()),
isStopped_(false),
phase_(phase),
parentPhase_(Timers::enter(phase)) {
}

void
Timer::stop() {
  if (isStopped_)
    return;
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  const double elapsedTime = elapsed();
  if (phase_ == PHASE_COUNT) {
    Timers::add(name_, elapsedTime);
  } else {
    // record before exit: the recursion rule reads the depth of the phase
    // while it is still on the stack.
    Timers::record(phase_, parentPhase_, elapsedTime);
    Timers::exit(phase_);
  }
#else
  if (phase_ != PHASE_COUNT)
    Timers::exit(phase_);
#endif
  isStopped_ = true;
}

Timer::~Timer() {
  if (!isStopped_)
    stop();
}

double Timer::elapsed() const {
  if (isStopped_) {
    return 0.;
  }
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startPoint_);
  return static_cast<double>(duration.count()) / 1000000;  // For the result in seconds
}

}  // namespace DYN
