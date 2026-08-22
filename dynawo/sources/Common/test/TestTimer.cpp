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

#include <chrono>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest_dynawo.h"
#include "DYNCommon.h"
#include "DYNTimer.h"

namespace DYN {

TEST(TimerTest, testPhaseAccumulation) {
  Timers::resetPhases();
  Timers::record(PHASE_RESIDUAL_EVAL, PHASE_COUNT, 0.5);
  Timers::record(PHASE_RESIDUAL_EVAL, PHASE_COUNT, 1.5);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_RESIDUAL_EVAL), 2.0);
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_RESIDUAL_EVAL), 2u);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseMinMs(PHASE_RESIDUAL_EVAL), 500.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseMaxMs(PHASE_RESIDUAL_EVAL), 1500.0);
}

TEST(TimerTest, testUntouchedPhaseIsZero) {
  Timers::resetPhases();
  Timers::record(PHASE_RESIDUAL_EVAL, PHASE_COUNT, 1.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_JACOBIAN_EVAL), 0.0);
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_JACOBIAN_EVAL), 0u);
}

TEST(TimerTest, testPhaseToStringCoversEveryPhase) {
  for (int i = 0; i < PHASE_COUNT; ++i) {
    const char* name = phaseToString(static_cast<TimerPhase>(i));
    ASSERT_TRUE(name != NULL);
    ASSERT_NE(std::string(name), "");
  }
}

TEST(TimerTest, testAccessorsAreSafeForPhaseCount) {
  // PHASE_COUNT is the documented "no parent" sentinel and is exactly what
  // enter() returns at the top level, so every accessor must tolerate it
  // (and any other out-of-range index) without indexing past the arrays.
  Timers::resetPhases();
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_COUNT), 0.0);
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_COUNT), 0u);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseMinMs(PHASE_COUNT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseMaxMs(PHASE_COUNT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_COUNT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_COUNT, PHASE_COUNT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_REINIT, PHASE_COUNT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_COUNT, PHASE_REINIT), 0.0);
}

TEST(TimerTest, testEnterReturnsParent) {
  Timers::resetPhases();
  ASSERT_EQ(Timers::enter(PHASE_SOLVER_STEP), PHASE_COUNT);
  ASSERT_EQ(Timers::enter(PHASE_NR_SOLVE), PHASE_SOLVER_STEP);
  Timers::exit(PHASE_NR_SOLVE);
  Timers::exit(PHASE_SOLVER_STEP);
}

TEST(TimerTest, testExclusiveExcludesChild) {
  Timers::resetPhases();
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);
  const TimerPhase pNr = Timers::enter(PHASE_NR_SOLVE);
  Timers::record(PHASE_NR_SOLVE, pNr, 0.8);
  Timers::exit(PHASE_NR_SOLVE);
  Timers::record(PHASE_SOLVER_STEP, pStep, 1.0);
  Timers::exit(PHASE_SOLVER_STEP);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_SOLVER_STEP), 1.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_SOLVER_STEP, PHASE_NR_SOLVE), 0.8);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_SOLVER_STEP), 0.2);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_NR_SOLVE), 0.8);
}

TEST(TimerTest, testExclusiveSubtractsEveryChildNotAChosenSubset) {
  // Regression coverage for phaseExclusive() subtracting an enumerated
  // subset of phases instead of looping over every one of them: give
  // PHASE_SOLVER_STEP a child recording under every other phase in the
  // enum, so no hand-picked enumeration, however large, can pass this.
  Timers::resetPhases();
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);

  const double childTime = 0.1;
  double sumOfChildren = 0.;
  for (int c = 0; c < PHASE_COUNT; ++c) {
    if (c == PHASE_SOLVER_STEP)
      continue;
    const TimerPhase child = static_cast<TimerPhase>(c);
    const TimerPhase pChild = Timers::enter(child);
    Timers::record(child, pChild, childTime);
    Timers::exit(child);
    sumOfChildren += childTime;
  }

  const double total = 10.0;
  Timers::record(PHASE_SOLVER_STEP, pStep, total);
  Timers::exit(PHASE_SOLVER_STEP);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_SOLVER_STEP), total - sumOfChildren);
}

TEST(TimerTest, testSelfEdgeNeverRecordedEvenWithoutNesting) {
  // Rule 3's self-edge exclusion is otherwise unreachable in test: at any
  // nesting depth greater than one, rule 2 returns before the edge code
  // runs, so a record with parent == phase and no enclosing enter() is the
  // only way to exercise it.
  Timers::resetPhases();
  Timers::record(PHASE_REINIT, PHASE_REINIT, 1.0);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_REINIT), 1.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_REINIT, PHASE_REINIT), 0.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_REINIT), 1.0);
}

TEST(TimerTest, testRecursionCountedOnce) {
  Timers::resetPhases();
  const TimerPhase pOuter = Timers::enter(PHASE_REINIT);
  const TimerPhase pInner = Timers::enter(PHASE_REINIT);
  Timers::record(PHASE_REINIT, pInner, 0.3);
  Timers::exit(PHASE_REINIT);
  Timers::record(PHASE_REINIT, pOuter, 1.0);
  Timers::exit(PHASE_REINIT);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseTotal(PHASE_REINIT), 1.0);
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_REINIT), 1u);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_REINIT, PHASE_REINIT), 0.0);
}

TEST(TimerTest, testChildOfRecursiveInstanceAttributesToThatPhase) {
  Timers::resetPhases();
  const TimerPhase pOuter = Timers::enter(PHASE_REINIT);
  Timers::enter(PHASE_REINIT);
  const TimerPhase pKin = Timers::enter(PHASE_KINSOL_SOLVE);
  Timers::record(PHASE_KINSOL_SOLVE, pKin, 0.4);
  Timers::exit(PHASE_KINSOL_SOLVE);
  Timers::exit(PHASE_REINIT);
  Timers::record(PHASE_REINIT, pOuter, 1.0);
  Timers::exit(PHASE_REINIT);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_REINIT, PHASE_KINSOL_SOLVE), 0.4);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_REINIT), 0.6);
}

TEST(TimerTest, testExclusiveNeverNegative) {
  Timers::resetPhases();
  const TimerPhase pLoop = Timers::enter(PHASE_SIMULATION_LOOP);
  const TimerPhase pSolve = Timers::enter(PHASE_SOLVER_SOLVE);
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);
  Timers::record(PHASE_SOLVER_STEP, pStep, 0.6);
  Timers::exit(PHASE_SOLVER_STEP);
  Timers::record(PHASE_SOLVER_SOLVE, pSolve, 0.9);
  Timers::exit(PHASE_SOLVER_SOLVE);
  Timers::record(PHASE_SIMULATION_LOOP, pLoop, 1.0);
  Timers::exit(PHASE_SIMULATION_LOOP);

  for (int i = 0; i < PHASE_COUNT; ++i)
    ASSERT_TRUE(Timers::instance().phaseExclusive(static_cast<TimerPhase>(i)) >= -1e-12);
}

TEST(TimerTest, testExclusiveSumEqualsSumOfAllRootTotals) {
  // The exclusive-time identity holds over every root, not just one:
  // Dynawo has more than one phase entered with no active parent
  // (PHASE_CALCULATE_IC and PHASE_SIMULATION_LOOP both start the stack
  // empty), so the sum of every phase's exclusive time must equal the sum
  // of every root's total, not the total of a single chosen root.
  Timers::resetPhases();
  const TimerPhase pIc = Timers::enter(PHASE_CALCULATE_IC);
  Timers::record(PHASE_CALCULATE_IC, pIc, 0.3);
  Timers::exit(PHASE_CALCULATE_IC);

  const TimerPhase pLoop = Timers::enter(PHASE_SIMULATION_LOOP);
  const TimerPhase pSolve = Timers::enter(PHASE_SOLVER_SOLVE);
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);
  Timers::record(PHASE_SOLVER_STEP, pStep, 0.6);
  Timers::exit(PHASE_SOLVER_STEP);
  Timers::record(PHASE_SOLVER_SOLVE, pSolve, 0.9);
  Timers::exit(PHASE_SOLVER_SOLVE);
  Timers::record(PHASE_SIMULATION_LOOP, pLoop, 1.0);
  Timers::exit(PHASE_SIMULATION_LOOP);

  double sum = 0.;
  for (int i = 0; i < PHASE_COUNT; ++i)
    sum += Timers::instance().phaseExclusive(static_cast<TimerPhase>(i));
  const double sumOfRoots = Timers::instance().phaseTotal(PHASE_CALCULATE_IC) +
      Timers::instance().phaseTotal(PHASE_SIMULATION_LOOP);
  ASSERT_DOUBLE_EQUALS_DYNAWO(sum, sumOfRoots);
}

TEST(TimerTest, testIndirectRecursionSetsCycleDetectedFlag) {
  // A -> B -> A: PHASE_SOLVER_STEP is already open when it is entered again
  // while PHASE_REINIT, not itself, is on top of the stack. This is the
  // indirect-recursion case enter() cannot attribute correctly; it must be
  // detected rather than silently mis-report.
  Timers::resetPhases();
  ASSERT_FALSE(Timers::instance().cycleDetected());
  Timers::enter(PHASE_SOLVER_STEP);
  Timers::enter(PHASE_REINIT);
  Timers::enter(PHASE_SOLVER_STEP);
  ASSERT_TRUE(Timers::instance().cycleDetected());
  Timers::exit(PHASE_SOLVER_STEP);
  Timers::exit(PHASE_REINIT);
  Timers::exit(PHASE_SOLVER_STEP);
}

TEST(TimerTest, testEnumTimerRecordsOnDestruction) {
  Timers::resetPhases();
  {
    Timer t(PHASE_MATRIX_COPY);
  }
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_MATRIX_COPY), 1u);
  ASSERT_TRUE(Timers::instance().phaseTotal(PHASE_MATRIX_COPY) >= 0.);
}

TEST(TimerTest, testEnumTimerNestingAttributesToParent) {
  Timers::resetPhases();
  {
    Timer outer(PHASE_SOLVER_STEP);
    {
      Timer inner(PHASE_NR_SOLVE);
      // Force inner's real duration past elapsed()'s microsecond truncation
      // floor so the recorded edge is deterministically nonzero, without a
      // scheduler-dependent sleep.
      const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(2);
      while (std::chrono::steady_clock::now() < until) {}
    }
  }
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_SOLVER_STEP), 1u);
  ASSERT_EQ(Timers::instance().phaseCalls(PHASE_NR_SOLVE), 1u);
  ASSERT_TRUE(Timers::instance().phaseParentChild(PHASE_SOLVER_STEP, PHASE_NR_SOLVE) > 0.);
  ASSERT_TRUE(Timers::instance().phaseExclusive(PHASE_SOLVER_STEP) >= -1e-12);
}

TEST(TimerTest, testCsvExportContentAndHeader) {
  Timers::resetPhases();
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);
  // NR_SOLVE is recorded twice with different durations (0.1 s and 0.4 s) so
  // its min and max genuinely differ: a same-unit min/max column swap in the
  // CSV writer would otherwise be undetectable, since a phase measured only
  // once has phaseMinMs() == phaseMaxMs() and a swap produces byte-identical
  // output.
  const TimerPhase pNr1 = Timers::enter(PHASE_NR_SOLVE);
  Timers::record(PHASE_NR_SOLVE, pNr1, 0.1);
  Timers::exit(PHASE_NR_SOLVE);
  const TimerPhase pNr2 = Timers::enter(PHASE_NR_SOLVE);
  Timers::record(PHASE_NR_SOLVE, pNr2, 0.4);
  Timers::exit(PHASE_NR_SOLVE);
  Timers::record(PHASE_SOLVER_STEP, pStep, 1.0);
  Timers::exit(PHASE_SOLVER_STEP);

  const std::string path = "phase_export_test.csv";
  ASSERT_TRUE(Timers::exportPhasesCSV(path));

  std::ifstream ifs(path.c_str());
  ASSERT_TRUE(ifs.is_open());
  std::string header;
  std::getline(ifs, header);
  ASSERT_EQ(header, "phase,total_seconds,exclusive_seconds,call_count,min_ms,max_ms");

  bool sawStep = false;
  bool sawNr = false;
  bool sawUntouched = false;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.find("SolverStep,") == 0) {
      sawStep = true;
      // total 1.0 s, exclusive 0.5 s (1.0 minus the two NR_SOLVE children,
      // 0.1 + 0.4), one call, min/max 1000 ms.
      ASSERT_EQ(line, "SolverStep,1.000000,0.500000,1,1000.0000,1000.0000");
    }
    if (line.find("NRSolve,") == 0) {
      sawNr = true;
      // total 0.5 s (0.1 + 0.4), exclusive 0.5 s (no children of its own),
      // two calls, min 100 ms, max 400 ms: min and max differ on purpose,
      // see the fixture comment above.
      ASSERT_EQ(line, "NRSolve,0.500000,0.500000,2,100.0000,400.0000");
    }
    if (line.find("MatrixCopy,") == 0)
      sawUntouched = true;
  }
  ASSERT_TRUE(sawStep);
  ASSERT_TRUE(sawNr);
  ASSERT_FALSE(sawUntouched);  // phases never entered are omitted
  ifs.close();
  std::remove(path.c_str());
}

TEST(TimerTest, testCsvExportFailsOnUnwritablePath) {
  Timers::resetPhases();
  Timers::record(PHASE_IO, PHASE_COUNT, 0.1);
  ASSERT_FALSE(Timers::exportPhasesCSV("/nonexistent-directory-xyz/out.csv"));
}

TEST(TimerTest, testKluSetupNestsUnderJacobianEval) {
  Timers::resetPhases();
  const TimerPhase pJac = Timers::enter(PHASE_JACOBIAN_EVAL);
  const TimerPhase pSetup = Timers::enter(PHASE_KLU_SETUP);
  Timers::record(PHASE_KLU_SETUP, pSetup, 0.4);
  Timers::exit(PHASE_KLU_SETUP);
  Timers::record(PHASE_JACOBIAN_EVAL, pJac, 1.0);
  Timers::exit(PHASE_JACOBIAN_EVAL);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_JACOBIAN_EVAL, PHASE_KLU_SETUP), 0.4);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_JACOBIAN_EVAL), 0.6);
}

TEST(TimerTest, testRootTotalMatchesSumOfExclusiveAcrossTwoRoots) {
  // rootTotal() must come from record_()'s own bookkeeping of parent ==
  // PHASE_COUNT measurements, not be derived after the fact: a mutation that
  // fails to accumulate it (stays 0), accumulates it on every record instead
  // of only root ones (would pick up the nested SOLVER_SOLVE time too), or
  // only tracks a single root instead of summing every root, all disagree
  // with at least one of the two assertions below.
  Timers::resetPhases();
  const TimerPhase pIc = Timers::enter(PHASE_CALCULATE_IC);
  Timers::record(PHASE_CALCULATE_IC, pIc, 0.3);
  Timers::exit(PHASE_CALCULATE_IC);

  const TimerPhase pLoop = Timers::enter(PHASE_SIMULATION_LOOP);
  const TimerPhase pSolve = Timers::enter(PHASE_SOLVER_SOLVE);
  Timers::record(PHASE_SOLVER_SOLVE, pSolve, 0.4);
  Timers::exit(PHASE_SOLVER_SOLVE);
  Timers::record(PHASE_SIMULATION_LOOP, pLoop, 1.0);
  Timers::exit(PHASE_SIMULATION_LOOP);

  double sumExclusive = 0.;
  for (int i = 0; i < PHASE_COUNT; ++i)
    sumExclusive += Timers::instance().phaseExclusive(static_cast<TimerPhase>(i));

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().rootTotal(), 1.3);
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().rootTotal(), sumExclusive);
}

TEST(TimerTest, testStackUnwindsOnThrow) {
  Timers::resetPhases();
  try {
    Timer outer(PHASE_SOLVER_STEP);
    throw std::runtime_error("unwind");
  } catch (const std::runtime_error&) {
    // the RAII destructor must have popped the stack
  }
  {
    Timer t(PHASE_IO);
  }
  // If the stack had leaked, PHASE_IO would be attributed under SOLVER_STEP.
  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseParentChild(PHASE_SOLVER_STEP, PHASE_IO), 0.0);
}

}  // namespace DYN
