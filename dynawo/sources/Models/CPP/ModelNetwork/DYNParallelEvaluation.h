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

/// @brief below this many tasks a parallel region costs more than it saves
static const std::size_t parallelEvaluationMinTasks = 256;

/**
 * @brief run body(i) for i in [0, n), in parallel when asked and available
 *
 * An exception leaving an OpenMP structured block is undefined behaviour, so
 * every iteration catches, the first exception is kept, and it is rethrown
 * once the loop has finished. Model code does throw DYNError, so this is a
 * contract rather than a precaution.
 *
 * The loop is index based because the tree compiles as C++11 and a pragma on
 * a range based for needs OpenMP 5.0.
 *
 * The schedule is static with a chunk size of 1, i.e. round robin over the
 * index range, rather than the default static schedule's contiguous blocks.
 * Both callers hand this function an index range where the heavy tasks are
 * grouped at the front: the voltage levels lead the component vector and the
 * branches trail it, and within the full component vector the voltage levels
 * are again the heavy entries. A contiguous-block static schedule would then
 * hand the first few threads nothing but voltage levels and the rest nothing
 * but light branches, an imbalance that grows with thread count rather than
 * with actual load. Round robin spreads that structural gradient evenly
 * across threads at compile-time scheduling cost, with no runtime work queue,
 * so a fixed chunk size is preferred over `schedule(dynamic)` here: the
 * imbalance is positional and known ahead of time, not a runtime variation
 * that would need dynamic load stealing to catch. This choice cannot affect
 * bit-identity: which thread executes which index never changes what any
 * index writes or how many index-local values get summed elsewhere.
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
