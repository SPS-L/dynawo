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
 * @file Solvers/Common/TestKLUTiming.cpp
 * @brief Unit tests for the KLU setup timing interception installed by SolverCommon
 *
 * Deliberately exercises the observable contract of installKLUTiming()/uninstallKLUTiming()
 * against a stub SUNLinearSolver, so an implementation that does nothing (or wraps the wrong
 * routine, or never restores the original) would fail here. Lives in this test binary, which
 * already links SUNDIALS, rather than in the Common test binary, which must not gain a
 * SUNDIALS dependency.
 */

#include <sundials/sundials_linearsolver.h>

#include "gtest_dynawo.h"
#include "DYNSolverCommon.h"

namespace DYN {

namespace {

int g_stubSetupCalls = 0;  ///< number of times stubSetup() has been called since it was last reset

/**
 * @brief stand-in for a real KLU setup routine
 *
 * Records that it was called and returns a sentinel value distinct from any SUNLS_* code, so a
 * caller can tell whether the wrapper actually called through to this routine.
 *
 * @return 42
 */
int stubSetup(SUNLinearSolver /*LS*/, SUNMatrix /*A*/) {
  ++g_stubSetupCalls;
  return 42;
}

/**
 * @brief build a minimal stub SUNLinearSolver whose only live op is setup
 *
 * @param ops storage for the ops struct, owned by the caller and kept alive as long as solver is used
 * @param solver storage for the solver struct, owned by the caller
 */
void makeStubSolver(_generic_SUNLinearSolver_Ops& ops, _generic_SUNLinearSolver& solver) {
  ops = _generic_SUNLinearSolver_Ops();
  ops.setup = stubSetup;
  solver = _generic_SUNLinearSolver();
  solver.ops = &ops;
}

}  // namespace

TEST(SolverKLUTimingTest, testInstallUninstallContract) {
  g_stubSetupCalls = 0;

  _generic_SUNLinearSolver_Ops ops;
  _generic_SUNLinearSolver solverStruct;
  makeStubSolver(ops, solverStruct);
  SUNLinearSolver LS = &solverStruct;

  ASSERT_TRUE(SolverCommon::installKLUTiming(LS));
  ASSERT_FALSE(SolverCommon::installKLUTiming(LS));  // already instrumented: no second wrapper
  ASSERT_FALSE(SolverCommon::installKLUTiming(NULL));

  // The wrapper must call through to the original setup routine and return its value unchanged:
  // this is what "the original setup routine is preserved" means in observable terms.
  ASSERT_EQ(LS->ops->setup(LS, NULL), 42);
  ASSERT_EQ(g_stubSetupCalls, 1);

  // Fill the registry with distinct instances until installation is refused, proving the
  // capacity bound is real and enforced rather than effectively unlimited.
  const int kFillerCount = 32;
  _generic_SUNLinearSolver_Ops fillerOps[kFillerCount];
  _generic_SUNLinearSolver fillerSolvers[kFillerCount];
  int installedFillers = 0;
  bool sawRefusal = false;
  for (int i = 0; i < kFillerCount; ++i) {
    makeStubSolver(fillerOps[i], fillerSolvers[i]);
    if (SolverCommon::installKLUTiming(&fillerSolvers[i])) {
      ++installedFillers;
    } else {
      sawRefusal = true;
      break;
    }
  }
  ASSERT_TRUE(sawRefusal);

  for (int i = 0; i < installedFillers; ++i)
    SolverCommon::uninstallKLUTiming(&fillerSolvers[i]);

  // The pointer must be restored after unregistration, not left pointing at the wrapper: a
  // reviewer or a later re-user of this address must see the real setup routine again.
  SolverCommon::uninstallKLUTiming(LS);
  ASSERT_EQ(LS->ops->setup, &stubSetup);

  // Uninstalling an instance that was never (or no longer) registered, or is NULL, must be a
  // safe no-op rather than a crash or a double-free of a registry slot.
  ASSERT_NO_THROW(SolverCommon::uninstallKLUTiming(LS));
  ASSERT_NO_THROW(SolverCommon::uninstallKLUTiming(NULL));
}

}  // namespace DYN
