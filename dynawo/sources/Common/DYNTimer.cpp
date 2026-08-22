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

#include <cstdlib>
#include <fstream>
#include <iomanip>
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
  phaseStack_.reserve(PHASE_COUNT);
  resetPhases_();
}

Timers::~Timers() {
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  for (const auto& timer : timers_)
    std::cout << "TIMER[" << timer.first << "] = " << timer.second << " seconds in " << nbAppels_[timer.first] << " calls" << std::endl;

  // Instance methods only below: instance() must not be re-entered while
  // this singleton is being torn down, see printPhaseReport_() and
  // exportPhasesCSV_().
  //
  // A destructor unwinding at thread_local teardown has nowhere to propagate
  // an exception to: letting one escape here would terminate the process.
  // Losing the timing report is strictly preferable, so swallow everything.
  //
  // This is a fallback only: the normal path is the explicit emitReport()
  // call from Simulation::terminate(), made while the job's own trace sinks
  // are still installed and followed by resetPhases(). When that explicit
  // path ran, phaseCalls_ is already back to all zero here and
  // emitReportIfAny_() is a no-op; this only fires for a timed process that
  // never called emitReport() at all.
  try {
    emitReportIfAny_();
  } catch (...) {
    // Deliberately empty: see the comment above.
  }
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
  rootTotal_ = 0.;
  phaseStack_.clear();
  cycleWarned_ = false;
}

TimerPhase
Timers::enter(const TimerPhase phase) {
  return instance().enter_(phase);
}

TimerPhase
Timers::enter_(const TimerPhase phase) {
  // Guard mirrors the read accessors (see testAccessorsAreSafeForPhaseCount):
  // phase indexes phaseDepth_ below, and PHASE_COUNT, the documented "no
  // parent" sentinel returned by this very function, is exactly the kind of
  // value that can flow back in here from a caller. Treat any out-of-range
  // phase as a no-op rather than writing past the array.
  if (phase >= PHASE_COUNT)
    return PHASE_COUNT;

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
  if (phase >= PHASE_COUNT)
    return;
  // Only pop the stack when its top actually is the phase being left: two
  // Timer objects whose lifetimes are not properly nested (for example
  // stopped out of order) must not make this pop the wrong phase off the
  // stack, only the depth bookkeeping below is unconditional on phase.
  if (!phaseStack_.empty() && phaseStack_.back() == phase)
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
  // Guard mirrors the read accessors: phase indexes every array below, and
  // this is a public entry point, so an out-of-range phase (PHASE_COUNT
  // included) must be a no-op rather than writing past the arrays: this used
  // to write phaseTime_[PHASE_COUNT], landing on top of phaseCalls_[0], and
  // phaseDepth_[PHASE_COUNT] read below without this guard would itself
  // already be past the array.
  if (phase >= PHASE_COUNT)
    return;

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

  // Rule 3: no edge at the top level, and never a self edge. Both of those
  // excluded cases leave the whole measurement in this phase's own exclusive
  // time with nothing subtracted anywhere else, so both accumulate into
  // rootTotal_ the same way: see the note on rootTotal() for why this must
  // be a plain else, not conditioned on parent == PHASE_COUNT, to keep the
  // sum-of-exclusives identity unconditional. parent < PHASE_COUNT (rather
  // than just parent != PHASE_COUNT) also guards parentChild_[parent][...]
  // against any other out-of-range parent, folding it into the same safe
  // "no parent" path instead of indexing past the array.
  if (parent < PHASE_COUNT && parent != phase)
    parentChild_[parent][phase] += time;
  else
    rootTotal_ += time;
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

double
Timers::rootTotal() const {
  return rootTotal_;
}

bool
Timers::cycleDetected() const {
  return cycleWarned_;
}

void
Timers::printPhaseReport() {
  instance().printPhaseReport_();
}

void
Timers::printPhaseReport_() const {
  std::stringstream ss;
  ss << "Phase timing\n";
  ss << "  phase                 total (s)   exclusive (s)      calls\n";
  int negativeExclusiveCount = 0;
  for (int i = 0; i < PHASE_COUNT; ++i) {
    const TimerPhase phase = static_cast<TimerPhase>(i);
    if (phaseCalls(phase) == 0)
      continue;
    const double exclusive = phaseExclusive(phase);
    // Not clamped on purpose: a negative exclusive time is the only
    // surviving signal that a phase's attribution broke (for example an
    // out-of-order Timer stop, see exit_()), and clamping would delete it.
    if (exclusive < 0.)
      ++negativeExclusiveCount;
    ss << "  " << std::left << std::setw(20) << phaseToString(phase)
       << std::right << std::fixed << std::setprecision(6)
       << std::setw(12) << phaseTotal(phase)  // seconds
       << std::setw(16) << exclusive  // seconds
       << std::setw(11) << phaseCalls(phase) << "\n";
  }

  // The sum of every phase's exclusive time equals the sum of every
  // measurement recorded at top level (rootTotal()), see the note on
  // rootTotal() and phaseExclusive(). This holds by construction for any
  // input, including a corrupted stack: record_() always adds each
  // measurement to exactly one of parentChild_ or rootTotal_, so the two
  // totals below can never disagree and their equality proves nothing about
  // whether this run's numbers are trustworthy. cycleDetected() and the
  // negative-exclusive count are the figures that can actually signal
  // trouble.
  double sumExclusive = 0.;
  for (int i = 0; i < PHASE_COUNT; ++i)
    sumExclusive += phaseExclusive(static_cast<TimerPhase>(i));
  ss << "  root total (s): " << std::fixed << std::setprecision(6) << rootTotal()
     << "   sum of exclusive (s): " << sumExclusive
     << "   cycle detected: " << (cycleDetected() ? "yes" : "no")
     << "   negative exclusive phases: " << negativeExclusiveCount << "\n";

  Trace::info() << ss.str() << Trace::endline;
}

bool
Timers::exportPhasesCSV(const std::string& path) {
  return instance().exportPhasesCSV_(path);
}

bool
Timers::exportPhasesCSV_(const std::string& path) const {
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open())
    return false;
  ofs << "phase,total_seconds,exclusive_seconds,call_count,min_ms,max_ms\n";
  for (int i = 0; i < PHASE_COUNT; ++i) {
    const TimerPhase phase = static_cast<TimerPhase>(i);
    if (phaseCalls(phase) == 0)
      continue;
    ofs << phaseToString(phase) << ","
        << std::fixed << std::setprecision(6) << phaseTotal(phase) << ","  // seconds
        << phaseExclusive(phase) << ","  // seconds
        << phaseCalls(phase) << ","
        << std::setprecision(4) << phaseMinMs(phase) << ","  // milliseconds
        << phaseMaxMs(phase) << "\n";  // milliseconds
  }
  ofs.close();
  return !ofs.fail();
}

void
Timers::emitReport() {
  instance().emitReportIfAny_();
}

bool
Timers::hasAnyPhase_() const {
  for (int i = 0; i < PHASE_COUNT; ++i) {
    if (phaseCalls_[i] > 0)
      return true;
  }
  return false;
}

void
Timers::emitReportIfAny_() const {
  if (!hasAnyPhase_())
    return;
  printPhaseReport_();
  const char* const out = std::getenv("DYNAWO_TIMERS_OUTPUT");
  if (out != NULL)
    exportPhasesCSV_(std::string(out));
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
  // Full precision: a duration_cast to microseconds truncates towards zero
  // and loses up to 1 microsecond per call, which dominates the measurement
  // for short, frequently called phases.
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - startPoint_).count();
}

}  // namespace DYN
