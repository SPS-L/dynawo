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
 * @brief Solver performance profiling framework - implementation
 */

#include "DYNSolverProfiler.h"
#include "DYNTrace.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

#ifdef __linux__
#include <cstdio>
#include <string>
#endif

namespace DYN {

const char* phaseToString(ProfilePhase phase) {
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
    default:                    return "Unknown";
  }
}

// --- PhaseStats ---

PhaseStats::PhaseStats() {
  reset();
}

void PhaseStats::reset() {
  totalTime = 0.0;
  minTime = std::numeric_limits<double>::max();
  maxTime = 0.0;
  callCount = 0;
  peakMemoryKB = 0;
}

double PhaseStats::avgTime() const {
  if (callCount == 0)
    return 0.0;
  return totalTime / static_cast<double>(callCount);
}

// --- SolverProfiler ---

SolverProfiler& SolverProfiler::instance() {
  static SolverProfiler inst;
  return inst;
}

SolverProfiler::SolverProfiler() : enabled_(false) {
#ifdef DYNAWO_PROFILING
  // Build-time activation: profiling is always on in profiling builds
  enabled_ = true;
#endif
  reset();
}

SolverProfiler::~SolverProfiler() {
  if (enabled_ && stats_[PHASE_SIMULATION_LOOP].callCount > 0) {
    // Auto-export on destruction if data was collected
    const char* envExport = std::getenv("DYNAWO_PROFILE_OUTPUT");
    if (envExport != NULL) {
      std::string path(envExport);
      if (path.size() > 5 && path.substr(path.size() - 5) == ".json") {
        exportJSON(path);
      } else {
        exportCSV(path);
      }
    }
  }
}

bool SolverProfiler::isEnabled() const {
  return enabled_;
}

void SolverProfiler::setEnabled(bool enabled) {
  enabled_ = enabled;
}

void SolverProfiler::reset() {
  for (int i = 0; i < PHASE_COUNT; ++i) {
    stats_[i].reset();
  }
  timestepRecords_.clear();
}

void SolverProfiler::record(ProfilePhase phase, double elapsedSeconds) {
  if (!enabled_)
    return;
  PhaseStats& s = stats_[phase];
  s.totalTime += elapsedSeconds;
  s.callCount++;
  if (elapsedSeconds < s.minTime)
    s.minTime = elapsedSeconds;
  if (elapsedSeconds > s.maxTime)
    s.maxTime = elapsedSeconds;
}

void SolverProfiler::recordWithMemory(ProfilePhase phase, double elapsedSeconds, uint64_t memoryKB) {
  if (!enabled_)
    return;
  record(phase, elapsedSeconds);
  if (memoryKB > stats_[phase].peakMemoryKB)
    stats_[phase].peakMemoryKB = memoryKB;
}

void SolverProfiler::recordTimestep(double simTime, double stepDurationMs, uint64_t memoryKB) {
  if (!enabled_)
    return;
  TimestepRecord rec;
  rec.simTime = simTime;
  rec.stepDurationMs = stepDurationMs;
  rec.memoryKB = memoryKB;
  timestepRecords_.push_back(rec);
}

const PhaseStats& SolverProfiler::getStats(ProfilePhase phase) const {
  return stats_[phase];
}

double SolverProfiler::totalTime() const {
  return stats_[PHASE_SIMULATION_LOOP].totalTime;
}

void SolverProfiler::printReport() const {
  if (!enabled_)
    return;

  std::ostringstream oss;
  oss << "\n";
  oss << "===============================================================\n";
  oss << "  DYNAWO SOLVER PERFORMANCE PROFILE\n";
  oss << "===============================================================\n";
  oss << std::left << std::setw(20) << "Phase"
      << std::right << std::setw(12) << "Total(s)"
      << std::setw(10) << "Calls"
      << std::setw(12) << "Avg(ms)"
      << std::setw(12) << "Min(ms)"
      << std::setw(12) << "Max(ms)"
      << std::setw(10) << "Pct(%)"
      << "\n";
  oss << "  NOTE: times are INCLUSIVE (nested phases accumulate in parent)\n";
  oss << "---------------------------------------------------------------\n";

  double totalSim = stats_[PHASE_SIMULATION_LOOP].totalTime;
  if (totalSim <= 0.0)
    totalSim = 1.0;  // avoid division by zero

  for (int i = 0; i < PHASE_COUNT; ++i) {
    const PhaseStats& s = stats_[i];
    if (s.callCount == 0)
      continue;
    double pct = (s.totalTime / totalSim) * 100.0;
    double minMs = (s.minTime < std::numeric_limits<double>::max()) ? s.minTime * 1000.0 : 0.0;
    oss << std::left << std::setw(20) << phaseToString(static_cast<ProfilePhase>(i))
        << std::right << std::fixed << std::setprecision(3)
        << std::setw(12) << s.totalTime
        << std::setw(10) << s.callCount
        << std::setw(12) << s.avgTime() * 1000.0
        << std::setw(12) << minMs
        << std::setw(12) << s.maxTime * 1000.0
        << std::setw(10) << std::setprecision(1) << pct
        << "\n";
  }

  oss << "---------------------------------------------------------------\n";

  // --- Exclusive time breakdown ---
  // Exclusive time = phase total - sum of its direct children.
  // Parent-child relationships (phases that are instrumented as nested sub-scopes):
  //   SimulationLoop -> CalculateIC, SolverSolve, CurvesUpdate, IO
  //   SolverSolve    -> SolverStep, NRSolve, DiscreteEval, ModeEval, Reinit
  //   SolverStep     -> ResidualEval, JacobianEval, RootEval
  //   JacobianEval   -> MatrixCopy, KLUSymbolic, KLUSetup
  //   NRSolve        -> KINSOLSolve
  //   KINSOLSolve    -> ResidualEval, JacobianEval
  // NOTE: Exclusive times assume single-parent nesting; shared phases
  //       (ResidualEval, JacobianEval) may be double-counted under mixed
  //       IDA+KINSOL runs — interpret SolverStep exclusive time with caution.
  auto excl = [&](ProfilePhase parent, std::initializer_list<ProfilePhase> children) -> double {
    double t = stats_[parent].totalTime;
    for (ProfilePhase c : children)
      t -= stats_[c].totalTime;
    return t < 0.0 ? 0.0 : t;  // clamp: clock skew can give tiny negatives
  };

  // Fix B-EXCL-CALC_IC: PHASE_CALCULATE_IC runs inside the PHASE_SIMULATION_LOOP
  // RAII scope (via Simulation::init() -> calculateICCommon()) and must be
  // subtracted from SimulationLoop's exclusive time so that IC solve cost is
  // not misattributed as loop-dispatch overhead.
  double exclSimLoop  = excl(PHASE_SIMULATION_LOOP,
      {PHASE_CALCULATE_IC, PHASE_SOLVER_SOLVE, PHASE_CURVES_UPDATE, PHASE_IO});
  double exclSolve    = excl(PHASE_SOLVER_SOLVE,
      {PHASE_SOLVER_STEP, PHASE_NR_SOLVE, PHASE_DISCRETE_EVAL, PHASE_MODE_EVAL, PHASE_REINIT});
  double exclStep     = excl(PHASE_SOLVER_STEP,
      {PHASE_RESIDUAL_EVAL, PHASE_JACOBIAN_EVAL, PHASE_ROOT_EVAL});
  double exclJac      = excl(PHASE_JACOBIAN_EVAL,
      {PHASE_MATRIX_COPY, PHASE_KLU_SYMBOLIC, PHASE_KLU_SETUP});
  double exclNR       = excl(PHASE_NR_SOLVE,   {PHASE_KINSOL_SOLVE});
  double exclKINSOL   = excl(PHASE_KINSOL_SOLVE, {PHASE_RESIDUAL_EVAL, PHASE_JACOBIAN_EVAL});

  oss << "\n  Exclusive times (phase cost minus direct children):\n";
  oss << "---------------------------------------------------------------\n";
  oss << std::left << std::setw(24) << "Phase"
      << std::right << std::setw(12) << "Excl(s)"
      << std::setw(10) << "Pct(%)\n";
  oss << "---------------------------------------------------------------\n";

  auto printExcl = [&](const char* name, double t) {
    if (t <= 0.0) return;
    oss << std::left << std::setw(24) << name
        << std::right << std::fixed << std::setprecision(3)
        << std::setw(12) << t
        << std::setw(10) << std::setprecision(1) << (t / totalSim * 100.0)
        << "\n";
  };

  printExcl("SimulationLoop",  exclSimLoop);
  printExcl("SolverSolve",     exclSolve);
  printExcl("SolverStep",      exclStep);
  printExcl("JacobianEval",    exclJac);
  printExcl("NRSolve",         exclNR);
  printExcl("KINSOLSolve",     exclKINSOL);

  oss << "---------------------------------------------------------------\n";
  oss << "Timesteps recorded: " << timestepRecords_.size() << "\n";

  // Memory summary: prefer peak from timestep series (populated every step)
  // over PhaseStats::peakMemoryKB (only set for PHASE_SIMULATION_LOOP).
  uint64_t peakMem = 0;
  for (size_t i = 0; i < timestepRecords_.size(); ++i) {
    if (timestepRecords_[i].memoryKB > peakMem)
      peakMem = timestepRecords_[i].memoryKB;
  }
  if (peakMem == 0) {
    // Fall back to PhaseStats if no timestep records (e.g., profiling build
    // without DYN_PROFILE_RECORD_TIMESTEP calls).
    for (int i = 0; i < PHASE_COUNT; ++i) {
      if (stats_[i].peakMemoryKB > peakMem)
        peakMem = stats_[i].peakMemoryKB;
    }
  }
  if (peakMem > 0)
    oss << "Peak RSS: " << std::fixed << std::setprecision(2) << (static_cast<double>(peakMem) / 1024.0) << " MB\n";

  oss << "===============================================================\n";

  Trace::info() << oss.str() << Trace::endline;
}

void SolverProfiler::exportCSV(const std::string& filename) const {
  if (!enabled_)
    return;

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
    return;

  // Fix B-CSV-PARSE: add explicit section-header comment lines so that
  // standard CSV parsers (pandas, Excel) can detect section boundaries.
  // The analyze_profile.py companion script uses these markers.
  ofs << "# PHASES\n";
  ofs << "phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb\n";
  for (int i = 0; i < PHASE_COUNT; ++i) {
    const PhaseStats& s = stats_[i];
    if (s.callCount == 0)
      continue;
    double minMs = (s.minTime < std::numeric_limits<double>::max()) ? s.minTime * 1000.0 : 0.0;
    ofs << phaseToString(static_cast<ProfilePhase>(i)) << ","
        << std::fixed << std::setprecision(6) << s.totalTime << ","
        << s.callCount << ","
        << std::setprecision(4) << s.avgTime() * 1000.0 << ","
        << minMs << ","
        << s.maxTime * 1000.0 << ","
        << s.peakMemoryKB << "\n";
  }

  // Timestep time series
  if (!timestepRecords_.empty()) {
    ofs << "\n";
    ofs << "# TIMESTEPS\n";
    ofs << "sim_time,step_duration_ms,memory_kb\n";
    for (size_t i = 0; i < timestepRecords_.size(); ++i) {
      const TimestepRecord& r = timestepRecords_[i];
      ofs << std::fixed << std::setprecision(6) << r.simTime << ","
          << std::setprecision(4) << r.stepDurationMs << ","
          << r.memoryKB << "\n";
    }
  }
}

void SolverProfiler::exportJSON(const std::string& filename) const {
  if (!enabled_)
    return;

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
    return;

  ofs << "{\n";
  ofs << "  \"phases\": [\n";
  bool first = true;
  for (int i = 0; i < PHASE_COUNT; ++i) {
    const PhaseStats& s = stats_[i];
    if (s.callCount == 0)
      continue;
    if (!first)
      ofs << ",\n";
    first = false;
    double minMs = (s.minTime < std::numeric_limits<double>::max()) ? s.minTime * 1000.0 : 0.0;
    ofs << "    {"
        << "\"name\": \"" << phaseToString(static_cast<ProfilePhase>(i)) << "\", "
        << "\"total_seconds\": " << std::fixed << std::setprecision(6) << s.totalTime << ", "
        << "\"call_count\": " << s.callCount << ", "
        << "\"avg_ms\": " << std::setprecision(4) << s.avgTime() * 1000.0 << ", "
        << "\"min_ms\": " << minMs << ", "
        << "\"max_ms\": " << s.maxTime * 1000.0 << ", "
        << "\"peak_memory_kb\": " << s.peakMemoryKB
        << "}";
  }
  ofs << "\n  ],\n";

  ofs << "  \"timesteps\": [\n";
  for (size_t i = 0; i < timestepRecords_.size(); ++i) {
    const TimestepRecord& r = timestepRecords_[i];
    if (i > 0)
      ofs << ",\n";
    ofs << "    {"
        << "\"sim_time\": " << std::fixed << std::setprecision(6) << r.simTime << ", "
        << "\"step_duration_ms\": " << std::setprecision(4) << r.stepDurationMs << ", "
        << "\"memory_kb\": " << r.memoryKB
        << "}";
  }
  ofs << "\n  ]\n";
  ofs << "}\n";
}

uint64_t SolverProfiler::getCurrentMemoryKB() {
#ifdef __linux__
  FILE* f = fopen("/proc/self/status", "r");
  if (f == NULL)
    return 0;
  char line[256];
  uint64_t vmRSS = 0;
  while (fgets(line, sizeof(line), f) != NULL) {
    if (strncmp(line, "VmRSS:", 6) == 0) {
      // Format: "VmRSS:    <value> kB"
      char* p = line + 6;
      while (*p == ' ' || *p == '\t')
        ++p;
      vmRSS = static_cast<uint64_t>(atol(p));
      break;
    }
  }
  fclose(f);
  return vmRSS;
#else
  return 0;
#endif
}

// --- PhaseTimer ---

PhaseTimer::PhaseTimer(ProfilePhase phase, bool trackMemory) :
  phase_(phase),
  trackMemory_(trackMemory),
  start_(std::chrono::high_resolution_clock::now()) {
}

PhaseTimer::~PhaseTimer() {
  double elapsedSec = elapsed();
  if (trackMemory_) {
    uint64_t memKB = SolverProfiler::getCurrentMemoryKB();
    SolverProfiler::instance().recordWithMemory(phase_, elapsedSec, memKB);
  } else {
    SolverProfiler::instance().record(phase_, elapsedSec);
  }
}

double PhaseTimer::elapsed() const {
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double>(now - start_).count();
}

}  // namespace DYN
