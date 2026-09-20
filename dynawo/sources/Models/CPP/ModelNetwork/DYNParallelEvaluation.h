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
 * are again the heavy entries. A contiguous-block static schedule hands the
 * first few threads nothing but voltage levels and the rest nothing but
 * light branches, an imbalance that grows with thread count rather than with
 * actual load; round robin spreads that structural gradient evenly but
 * scatters each thread's indices and can cost cache locality.
 *
 * This trade-off was measured rather than argued: schedule(runtime) was
 * built once and OMP_SCHEDULE swept over static, static with a 64-chunk,
 * dynamic with a 32-chunk and guided, three replicates each, on pfr_400s at
 * networkEvaluationThreads=8 (reports/parallel_evaluation_2026-09-20/,
 * increment1_matrix.txt PART 5). None of the four recovers anything close
 * to the threads=1 serial worst-second range (median 1107.7 ms, 1093.6 to
 * 1155.7 ms across 6 runs): the best single parallel observation of any
 * schedule was 1211.1 ms, so the worst-second regression against serial
 * evaluation is not a scheduling artefact. Among the parallel schedules,
 * round robin (median worst second 1258.2 ms, range 1246.6 to 1281.3 ms
 * across 6 runs) has the best combination of typical performance and tail
 * behaviour: plain contiguous static had a lower median (1226.6 ms) but a
 * worst observed value of 1479.8 ms with 10 over-budget seconds, worse than
 * anything round robin produced; static with a 64 chunk was similarly
 * unstable (up to 1517.8 ms); dynamic with a 32 chunk was the most
 * consistent (a 5 ms spread across 3 runs) but its median was about 63 ms
 * worse than round robin's; guided was uniformly worse on every run. Round
 * robin is kept as the fixed schedule on that basis, not on the balance
 * argument alone. This choice cannot affect bit-identity: which thread
 * executes which index never changes what any index writes or how many
 * index-local values get summed elsewhere, and curve output was confirmed
 * bit-identical across all five schedules measured.
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
