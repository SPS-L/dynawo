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
  // This is the M2 regression: SOLVER_STEP has three children here and all
  // three must be subtracted, not an enumerated subset of them.
  Timers::resetPhases();
  const TimerPhase pStep = Timers::enter(PHASE_SOLVER_STEP);

  const TimerPhase pNr = Timers::enter(PHASE_NR_SOLVE);
  Timers::record(PHASE_NR_SOLVE, pNr, 0.5);
  Timers::exit(PHASE_NR_SOLVE);

  const TimerPhase pRoot = Timers::enter(PHASE_ROOT_EVAL);
  Timers::record(PHASE_ROOT_EVAL, pRoot, 0.2);
  Timers::exit(PHASE_ROOT_EVAL);

  const TimerPhase pMode = Timers::enter(PHASE_MODE_EVAL);
  Timers::record(PHASE_MODE_EVAL, pMode, 0.1);
  Timers::exit(PHASE_MODE_EVAL);

  Timers::record(PHASE_SOLVER_STEP, pStep, 1.0);
  Timers::exit(PHASE_SOLVER_STEP);

  ASSERT_DOUBLE_EQUALS_DYNAWO(Timers::instance().phaseExclusive(PHASE_SOLVER_STEP), 0.2);
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

TEST(TimerTest, testExclusiveSumEqualsRootTotal) {
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

  double sum = 0.;
  for (int i = 0; i < PHASE_COUNT; ++i)
    sum += Timers::instance().phaseExclusive(static_cast<TimerPhase>(i));
  ASSERT_DOUBLE_EQUALS_DYNAWO(sum, Timers::instance().phaseTotal(PHASE_SIMULATION_LOOP));
}

}  // namespace DYN
