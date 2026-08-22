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

}  // namespace DYN
