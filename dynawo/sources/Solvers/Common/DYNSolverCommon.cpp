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
#include <sunmatrix/sunmatrix_sparse.h>
#include <sunlinsol/sunlinsol_klu.h>

#include "DYNMacrosMessage.h"
#include "DYNModel.h"
#include "DYNSolverCommon.h"
#include "DYNSparseMatrix.h"
#include "DYNTimer.h"
#include "DYNTrace.h"

namespace {

#if defined(_DEBUG_) || defined(PRINT_TIMERS)
/**
 * @brief association between a KLU linear solver and its original setup routine
 */
struct KLUSetupEntry {
  SUNLinearSolver solver;  ///< instrumented solver
  int (*origSetup)(SUNLinearSolver, SUNMatrix);  ///< routine replaced by the wrapper
};

const size_t MAX_KLU_SOLVERS = 16;  ///< headroom above the live-instance count seen in practice, not itself the fix for exhaustion
// thread_local to match Timers::instance(): concurrent construction on two threads must not race on a shared count or
// overwrite each other's entries, which would otherwise leave the losing solver wrapped with no registry entry.
thread_local KLUSetupEntry g_kluTable[MAX_KLU_SOLVERS];  ///< live instrumented solvers, this thread only
thread_local size_t g_kluCount = 0;  ///< number of live entries in g_kluTable
thread_local bool g_kluCapacityWarned = false;  ///< one-shot guard for the table-full warning, this thread only

/**
 * @brief timed replacement for a KLU solver's setup routine
 *
 * @param LS linear solver being set up
 * @param A matrix to factorise
 * @return the original routine's return code, or a hard failure if LS has no registry entry
 */
int profiledKLUSetup(SUNLinearSolver LS, SUNMatrix A) {
  for (size_t i = 0; i < g_kluCount; ++i) {
    if (g_kluTable[i].solver == LS) {
      DYNAWO_TIMER_PHASE(PHASE_KLU_SETUP);
      return g_kluTable[i].origSetup(LS, A);
    }
  }
  // The wrapper is installed on LS->ops->setup but the registry entry is gone: reachable only if the entry was
  // evicted or never recorded, both bugs elsewhere in this file. Returning SUNLS_SUCCESS here without calling any
  // setup routine would let KINSOL or IDA proceed against a stale or uninitialised factorisation, so fail hard.
  DYN::Trace::warn() << "DYNSolverCommon: KLU setup wrapper invoked on a solver with no registry entry, "
                      << "refusing to factorise instead of silently skipping it" << DYN::Trace::endline;
  return SUNLS_PACKAGE_FAIL_UNREC;
}
#endif

}  // namespace

namespace DYN {

bool
SolverCommon::installKLUTiming(SUNLinearSolver LS) {
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  if (LS == NULL || LS->ops == NULL || LS->ops->setup == NULL)
    return false;
  if (LS->ops->setup == profiledKLUSetup)
    return false;  // already instrumented
  if (g_kluCount >= MAX_KLU_SOLVERS) {
    if (!g_kluCapacityWarned) {
      Trace::warn() << "DYNSolverCommon: KLU timing registry is full, this solver instance will not be timed"
                    << Trace::endline;
      g_kluCapacityWarned = true;
    }
    return false;
  }
  g_kluTable[g_kluCount].solver = LS;
  g_kluTable[g_kluCount].origSetup = LS->ops->setup;
  ++g_kluCount;
  LS->ops->setup = profiledKLUSetup;
  return true;
#else
  (void)LS;
  return false;
#endif
}

void
SolverCommon::uninstallKLUTiming(SUNLinearSolver LS) {
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  for (size_t i = 0; i < g_kluCount; ++i) {
    if (g_kluTable[i].solver == LS) {
      if (LS != NULL && LS->ops != NULL)
        LS->ops->setup = g_kluTable[i].origSetup;
      g_kluTable[i] = g_kluTable[g_kluCount - 1];
      --g_kluCount;
      return;
    }
  }
#else
  (void)LS;
#endif
}

bool
SolverCommon::copySparseToKINSOL(const SparseMatrix& smj, SUNMatrix& JJ, const int& size, sunindextype * lastRowVals) {
  DYNAWO_TIMER_PHASE(PHASE_MATRIX_COPY);
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
      DYNAWO_TIMER_PHASE(PHASE_KLU_SYMBOLIC);
      SUNLinSol_KLUReInit(LS, JJ, SM_NNZ_S(JJ), 2);  // reinit symbolic factorisation
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
