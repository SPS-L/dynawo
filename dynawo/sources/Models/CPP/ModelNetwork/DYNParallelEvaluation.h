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
 * @file  DYNParallelEvaluation.h
 *
 * @brief Index loop that may run in parallel, with a deferred rethrow.
 *
 */
#ifndef MODELS_CPP_MODELNETWORK_DYNPARALLELEVALUATION_H_
#define MODELS_CPP_MODELNETWORK_DYNPARALLELEVALUATION_H_

#include <cstddef>
#include <exception>

namespace DYN {

/// @brief task-count floor below which callers should run serially instead of through parallelFor
static const std::size_t parallelEvaluationMinTasks = 256;

/**
 * @brief run body(i) for i in [0, n), in parallel when asked and available
 *
 * The loop is index based. It runs in parallel only when nbThreads is
 * greater than 1 and the tree is built with OpenMP; otherwise it runs
 * serially. When parallel, the schedule is static with a chunk size of 1,
 * i.e. round robin over the index range, rather than the default static
 * schedule's contiguous blocks.
 *
 * An exception thrown by any iteration's body(i) is caught; the first one
 * raised is kept and rethrown once the loop has finished, rather than being
 * allowed to escape the parallel region.
 *
 * @param n number of iterations
 * @param nbThreads thread count; 1 means run serially
 * @param body callable invoked as body(i)
 */
template <typename Body>
void parallelFor(const int n, const unsigned nbThreads, Body body) {
  std::exception_ptr firstError;
#ifdef _OPENMP
#pragma omp parallel for schedule(static, 1) num_threads(nbThreads) if (nbThreads > 1)
#else
  (void) nbThreads;
#endif
  for (int i = 0; i < n; ++i) {
    try {
      body(i);
    } catch (...) {
#ifdef _OPENMP
#pragma omp critical(DYNParallelEvaluationError)
#endif
      {
        if (!firstError)
          firstError = std::current_exception();
      }
    }
  }
  if (firstError)
    std::rethrow_exception(firstError);
}

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORK_DYNPARALLELEVALUATION_H_
