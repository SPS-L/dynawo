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

#include <stdexcept>
#include <vector>

#include "gtest_dynawo.h"
#include "DYNParallelEvaluation.h"
#include "DYNModelNetwork.h"

namespace DYN {

TEST(ModelsModelNetwork, ParallelForVisitsEveryIndexOnce) {
  const int n = 1000;
  std::vector<int> seen(n, 0);

  parallelFor(n, 4, [&seen](const int i) { seen[i] += 1; });

  for (int i = 0; i < n; ++i)
    ASSERT_EQ(seen[i], 1);
}

TEST(ModelsModelNetwork, ParallelForRunsSeriallyForOneThread) {
  const int n = 10;
  std::vector<int> order;

  parallelFor(n, 1, [&order](const int i) { order.push_back(i); });

  ASSERT_EQ(order.size(), static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
    ASSERT_EQ(order[i], i);
}

TEST(ModelsModelNetwork, ParallelForRethrowsAfterTheLoop) {
  const int n = 100;
  std::vector<int> seen(n, 0);

  ASSERT_THROW(
      parallelFor(n, 4, [&seen](const int i) {
        seen[i] += 1;
        if (i == 7)
          throw std::runtime_error("boom");
      }),
      std::runtime_error);

  // Every iteration still ran: the exception is deferred, not propagated out
  // of the structured block, so it cannot abort the other threads.
  for (int i = 0; i < n; ++i)
    ASSERT_EQ(seen[i], 1);
}

TEST(ModelsModelNetwork, EvaluationThreadsDefaultsToOne) {
  ModelNetwork network;

  ASSERT_EQ(network.getEvaluationThreads(), 1u);
}

TEST(ModelsModelNetwork, EffectiveThreadsFallsBackToSerialForSmallWork) {
  ModelNetwork network;
  network.setEvaluationThreads(8);

  ASSERT_EQ(network.effectiveThreads(parallelEvaluationMinTasks), 8u);
  ASSERT_EQ(network.effectiveThreads(parallelEvaluationMinTasks - 1), 1u);
}

}  // namespace DYN
