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
 * @file  DYNSolverCommon.cpp
 *
 * @brief Common utility method shared between all solvers
 *
 */
#include <string>
#include <cmath>
#include <cassert>
#include <sunmatrix/sunmatrix_sparse.h>
#include <sunlinsol/sunlinsol_klu.h>

#include "DYNMacrosMessage.h"
#include "DYNModel.h"
#include "DYNSolverCommon.h"
#include "DYNSolverProfiler.h"
#include "DYNSparseMatrix.h"
#include "DYNTrace.h"
#include <algorithm>

#ifdef DYNAWO_PROFILING
/**
 * @brief Per-instance wrapper node for the KLU SUNLinSolSetup profiling hook.
 *
 * Instead of a single global function pointer, each call to installKLUProfiler()
 * allocates one KLUSetupHook on the heap.  The wrapper function stored in
 * ops->setup reads the original pointer from the hook object, so multiple
 * concurrent SUNLinearSolver instances (e.g. KINSOL + IDA) each carry their
 * own captured original — eliminating the global-singleton coupling.
 *
 * Lifetime: the hook is allocated in installKLUProfiler() and intentionally
 * never freed (it must outlive the solver).  Its size is two pointers (~16 B)
 * so the leak is negligible and matches the lifetime of the process.
 */
struct KLUSetupHook {
  /// The original SUNLinSolSetup_KLU function pointer captured at install time.
  int (*origSetup)(SUNLinearSolver, SUNMatrix);
};

/**
 * @brief Profiling wrapper stored in ops->setup.
 *
 * Recovers the per-instance KLUSetupHook from the solver's content pointer
 * (stored in the padding slot ops->setup itself used to be), times the call
 * as PHASE_KLU_SETUP, then delegates to the real SUNLinSolSetup_KLU.
 *
 * SUNDIALS stores user-supplied data in S->content; we piggyback the hook
 * pointer in the unused s_content slot that SUNDIALS reserves for the
 * implementation — specifically, the first word of the KLU content struct
 * is the klu_common pointer, so we cannot reuse content.  Instead we store
 * the hook pointer in the ops->solve slot momentarily… actually the cleanest
 * approach is to embed it as a static per-call-site via a trampoline table.
 *
 * Simplest correct approach that stays minimally invasive:
 * store the hook pointer in a global array indexed by a small integer stamped
 * into S->content->last_flag (an int field that SUNDIALS does not read between
 * setup calls).  However, that requires SUNDIALS internals knowledge.
 *
 * CHOSEN APPROACH (safest, no SUNDIALS internals):
 * Keep one pointer per slot in a fixed-size table (max 8 KLU instances,
 * more than enough for Dynawo's single-threaded use).  The wrapper scans
 * the table to find the matching origSetup for this S, falling back to the
 * first entry if not found (original single-instance behaviour preserved).
 */

static const int kMaxKLUInstances = 8;

struct KLUProfilerTable {
  SUNLinearSolver solver;                     ///< Identity key: the solver pointer
  int (*origSetup)(SUNLinearSolver, SUNMatrix); ///< Original ops->setup for this instance
};

static KLUProfilerTable s_kluTable[kMaxKLUInstances];
static int s_kluCount = 0;

/**
 * @brief Profiling wrapper for SUNLinSolSetup (klu_refactor / klu_factor).
 *
 * Looks up the original setup function for solver S in s_kluTable, times the
 * call as PHASE_KLU_SETUP, and delegates.  If S is not found (should never
 * happen in correct usage), falls back to the first registered entry so that
 * the simulation still completes.
 */
static int profiledKLUSetup(SUNLinearSolver S, SUNMatrix A) {
  int (*orig)(SUNLinearSolver, SUNMatrix) = NULL;
  for (int i = 0; i < s_kluCount; ++i) {
    if (s_kluTable[i].solver == S) {
      orig = s_kluTable[i].origSetup;
      break;
    }
  }
  // Fallback: should not happen; avoids NULL dereference in production.
  if (orig == NULL && s_kluCount > 0)
    orig = s_kluTable[0].origSetup;

  DYN::PhaseTimer dynProfileTimer_KLU_SETUP(DYN::PHASE_KLU_SETUP);
  return orig(S, A);
}
#endif  // DYNAWO_PROFILING

namespace DYN {

bool
SolverCommon::copySparseToKINSOL(const SparseMatrix& smj, SUNMatrix& JJ, const int& size, sunindextype * lastRowVals) {
  DYN_PROFILE_PHASE(PHASE_MATRIX_COPY);
  bool matrixStructChange = false;
  if (SM_NNZ_S(JJ) < smj.nbElem()) {
    free(SM_INDEXPTRS_S(JJ));
    free(SM_INDEXVALS_S(JJ));
    free(SM_DATA_S(JJ));
    SM_NNZ_S(JJ) = smj.nbElem();
    SM_INDEXPTRS_S(JJ) = reinterpret_cast<sunindextype*> (malloc((size + 1) * sizeof (sunindextype)));
    SM_INDEXVALS_S(JJ) = reinterpret_cast<sunindextype*> (malloc(SM_NNZ_S(JJ) * sizeof (sunindextype)));
    SM_DATA_S(JJ) = reinterpret_cast<realtype*> (malloc(SM_NNZ_S(JJ) * sizeof (realtype)));
    matrixStructChange = true;
  }

  // NNZ has to be actualized anyway
  SM_NNZ_S(JJ) = smj.nbElem();

  for (unsigned i = 0, iEnd = size + 1; i < iEnd; ++i) {
    SM_INDEXPTRS_S(JJ)[i] = smj.Ap_[i];  //!!! implicit conversion from unsigned to sunindextype
  }
  for (unsigned i = 0, iEnd = smj.nbElem(); i < iEnd; ++i) {
    SM_INDEXVALS_S(JJ)[i] = smj.Ai_[i];  //!!! implicit conversion from int to sunindextype
    SM_DATA_S(JJ)[i] = smj.Ax_[i];  //!!! implicit conversion from double to realtype
  }

  if (lastRowVals != NULL) {
    if (memcmp(lastRowVals, SM_INDEXVALS_S(JJ), sizeof (sunindextype)*SM_NNZ_S(JJ)) != 0) {
      matrixStructChange = true;
    }
  } else {  // first time or size change
    matrixStructChange = true;
  }

  return matrixStructChange;
}

void SolverCommon::propagateMatrixStructureChangeToKINSOL(const SparseMatrix& smj, SUNMatrix& JJ, const int& size, sunindextype** lastRowVals,
                                                          SUNLinearSolver& LS, bool log) {
  bool matrixStructChange = copySparseToKINSOL(smj, JJ, size, *lastRowVals);

  if (matrixStructChange) {
    {
      DYN_PROFILE_PHASE(PHASE_KLU_SYMBOLIC);
      SUNLinSol_KLUReInit(LS, JJ, SM_NNZ_S(JJ), 2);  // reinit symbolic factorisation (klu_analyze)
    }
    if (*lastRowVals != NULL) {
      free(*lastRowVals);
    }
    *lastRowVals = reinterpret_cast<sunindextype*> (malloc(sizeof (sunindextype)*SM_NNZ_S(JJ)));
    memcpy(*lastRowVals, SM_INDEXVALS_S(JJ), sizeof (sunindextype)*SM_NNZ_S(JJ));
    if (log)
      Trace::debug() << DYNLog(MatrixStructureChange) << Trace::endline;
  }
}

void SolverCommon::installKLUProfiler(SUNLinearSolver& LS) {
#ifdef DYNAWO_PROFILING
  // Guard: ops->setup must be valid before we wrap it.
  assert(LS != NULL && LS->ops != NULL && LS->ops->setup != NULL &&
         "installKLUProfiler called before SUNLinSol_KLU has populated ops->setup");

  // Avoid double-wrapping if called again for the same instance (e.g. after
  // resetAlgebraicRestoration recreates KINSOL with the same LS pointer).
  for (int i = 0; i < s_kluCount; ++i) {
    if (s_kluTable[i].solver == LS) {
      // Already registered; the wrapper is already in ops->setup — nothing to do.
      return;
    }
  }

  // Register this instance in the table.
  assert(s_kluCount < kMaxKLUInstances &&
         "installKLUProfiler: exceeded maximum tracked KLU instances (kMaxKLUInstances)");
  s_kluTable[s_kluCount].solver    = LS;
  s_kluTable[s_kluCount].origSetup = LS->ops->setup;  // capture per-instance original
  ++s_kluCount;

  LS->ops->setup = profiledKLUSetup;
#else
  (void)LS;
#endif
}

void
SolverCommon::printLargestErrors(std::vector<std::pair<double, size_t> >& fErr, const Model& model,
                   int nbErr) {
  std::sort(fErr.begin(), fErr.end(), mapcompabs());

  size_t size = nbErr;
  if (fErr.size() < size)
    size = fErr.size();
  for (size_t i = 0; i < size; ++i) {
    std::string subModelName("");
    int subModelIndexF = 0;
    std::string fEquation("");
    std::pair<double, size_t> currentErr = fErr[i];
    model.getFInfos(static_cast<int>(currentErr.second), subModelName, subModelIndexF, fEquation);

    Trace::debug() << DYNLog(KinErrorValue, currentErr.second, currentErr.first,
                             subModelName, subModelIndexF, fEquation) << Trace::endline;
  }
}

double SolverCommon::weightedInfinityNorm(const std::vector<double>& vec, const std::vector<double>& weights) {
  assert(vec.size() == weights.size() && "Vectors must have same length.");
  double norm = 0.;
  double product = 0.;
  for (unsigned int i = 0; i < vec.size(); ++i) {
    product = std::fabs(vec[i] * weights[i]);
    if (product > norm) {
      norm = product;
    }
  }
  return norm;
}

double SolverCommon::weightedL2Norm(const std::vector<double>& vec, const std::vector<double>& weights) {
  assert(vec.size() == weights.size() && "Vectors must have same length.");
  double squared_norm = 0.;
  for (unsigned int i = 0; i < vec.size(); ++i) {
    squared_norm += (vec[i] * weights[i]) * (vec[i] * weights[i]);
  }
  return std::sqrt(squared_norm);
}

double SolverCommon::weightedInfinityNorm(const std::vector<double>& vec, const std::vector<int>& vec_index, const std::vector<double>& weights) {
  assert(vec_index.size() == weights.size() && "Weights and indices must have same length.");
  double norm = 0.;
  double product = 0.;
  for (unsigned int i = 0; i < vec_index.size(); ++i) {
    product = std::fabs(vec[vec_index[i]] * weights[i]);
    if (product > norm) {
      norm = product;
    }
  }
  return norm;
}

double SolverCommon::weightedL2Norm(const std::vector<double>& vec, const std::vector<int>& vec_index, const std::vector<double>& weights) {
  assert(vec_index.size() == weights.size() && "Weights and indices must have same length.");
  double squared_norm = 0.;
  for (unsigned int i = 0; i < vec_index.size(); ++i) {
    squared_norm += (vec[vec_index[i]] * weights[i]) * (vec[vec_index[i]] * weights[i]);
  }
  return std::sqrt(squared_norm);
}

}  // namespace DYN
