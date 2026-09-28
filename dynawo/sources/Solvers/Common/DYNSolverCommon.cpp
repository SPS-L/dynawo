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
#include "DYNTrace.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <utility>
#include <vector>

namespace {

// Phase 0 experiment 1 (throwaway): a union-pattern cache in front of KLU.
// Enabled by the environment variable DYNAWO_PATTERN_CACHE. The pattern handed to
// KLU is the union of every pattern seen since the last symbolic analysis, with
// explicit zeros where the current evaluation dropped an entry, so that
// klu_refactor applies whenever no position outside the union appears.
bool patternCacheEnabled() {
  static const bool enabled = (std::getenv("DYNAWO_PATTERN_CACHE") != NULL);
  return enabled;
}

struct UnionPattern {
  int size;
  std::vector<sunindextype> Ap;  // size + 1 column pointers of the union
  std::vector<sunindextype> Ai;  // row indices of the union, sorted within a column
  long evaluations;
  long reanalyses;
  UnionPattern() : size(0), evaluations(0), reanalyses(0) {}
};

std::map<SUNLinearSolver, UnionPattern>& unionPatterns() {
  static std::map<SUNLinearSolver, UnionPattern> patterns;
  return patterns;
}

typedef std::pair<sunindextype, double> Entry;

void propagateWithUnionPattern(const DYN::SparseMatrix& smj, SUNMatrix& JJ, const int& size, SUNLinearSolver& LS, bool log) {
  UnionPattern& up = unionPatterns()[LS];
  ++up.evaluations;
  const bool first = (up.size != size);
  if (first) {
    up.size = size;
    up.Ap.assign(size + 1, 0);
    up.Ai.clear();
  }

  // 1. The current pattern, sorted by row within each column.
  std::vector<Entry> cur;
  cur.reserve(smj.nbElem());
  std::vector<sunindextype> curAp(size + 1);
  bool duplicates = false;
  for (int j = 0; j < size; ++j) {
    curAp[j] = static_cast<sunindextype>(cur.size());
    for (unsigned k = smj.Ap_[j]; k < smj.Ap_[j + 1]; ++k)
      cur.push_back(Entry(static_cast<sunindextype>(smj.Ai_[k]), smj.Ax_[k]));
    std::sort(cur.begin() + curAp[j], cur.end());
    for (size_t k = curAp[j] + 1; k < cur.size(); ++k)
      if (cur[k].first == cur[k - 1].first) duplicates = true;
  }
  curAp[size] = static_cast<sunindextype>(cur.size());
  if (duplicates) {
    static bool warned = false;
    if (!warned) {
      std::fprintf(stderr, "DYNAWO_PATTERN_CACHE: duplicate row indices within a column; first value kept\n");
      warned = true;
    }
  }

  // 2. Merge the current pattern into the union; any new position changes the structure.
  std::vector<sunindextype> mergedAp(size + 1);
  std::vector<sunindextype> mergedAi;
  mergedAi.reserve(up.Ai.size() + cur.size());
  bool changed = first;
  for (int j = 0; j < size; ++j) {
    mergedAp[j] = static_cast<sunindextype>(mergedAi.size());
    sunindextype a = up.Ap[j];
    const sunindextype aEnd = up.Ap[j + 1];
    sunindextype c = curAp[j];
    const sunindextype cEnd = curAp[j + 1];
    while (a < aEnd || c < cEnd) {
      if (c >= cEnd || (a < aEnd && up.Ai[a] < cur[c].first)) {
        mergedAi.push_back(up.Ai[a]);
        ++a;
      } else if (a >= aEnd || cur[c].first < up.Ai[a]) {
        mergedAi.push_back(cur[c].first);
        ++c;
        changed = true;
        while (c < cEnd && cur[c].first == mergedAi.back()) ++c;  // skip duplicates
      } else {
        mergedAi.push_back(up.Ai[a]);
        ++a;
        ++c;
        while (c < cEnd && cur[c].first == mergedAi.back()) ++c;  // skip duplicates
      }
    }
  }
  mergedAp[size] = static_cast<sunindextype>(mergedAi.size());
  if (changed) {
    up.Ap.swap(mergedAp);
    up.Ai.swap(mergedAi);
  }

  // 3. Scatter the current values into the union layout, explicit zeros elsewhere.
  const sunindextype unnz = up.Ap[size];
  if (SM_NNZ_S(JJ) < unnz) {
    free(SM_INDEXPTRS_S(JJ));
    free(SM_INDEXVALS_S(JJ));
    free(SM_DATA_S(JJ));
    SM_INDEXPTRS_S(JJ) = reinterpret_cast<sunindextype*> (malloc((size + 1) * sizeof (sunindextype)));
    SM_INDEXVALS_S(JJ) = reinterpret_cast<sunindextype*> (malloc(unnz * sizeof (sunindextype)));
    SM_DATA_S(JJ) = reinterpret_cast<realtype*> (malloc(unnz * sizeof (realtype)));
  }
  SM_NNZ_S(JJ) = unnz;
  for (int j = 0; j <= size; ++j)
    SM_INDEXPTRS_S(JJ)[j] = up.Ap[j];
  for (sunindextype k = 0; k < unnz; ++k) {
    SM_INDEXVALS_S(JJ)[k] = up.Ai[k];
    SM_DATA_S(JJ)[k] = 0.;
  }
  for (int j = 0; j < size; ++j) {
    sunindextype a = up.Ap[j];
    const sunindextype aEnd = up.Ap[j + 1];
    for (sunindextype c = curAp[j]; c < curAp[j + 1]; ++c) {
      while (a < aEnd && up.Ai[a] < cur[c].first) ++a;
      if (a < aEnd && up.Ai[a] == cur[c].first) {
        SM_DATA_S(JJ)[a] = cur[c].second;  // duplicates: last value wins, warned above
      }
    }
  }

  if (changed) {
    ++up.reanalyses;
    SUNLinSol_KLUReInit(LS, JJ, unnz, 2);  // reinit symbolic factorisation on the union pattern
    std::fprintf(stderr, "DYNAWO_PATTERN_CACHE: reanalysis %ld at evaluation %ld, union nnz %ld, current nnz %d, solver %p\n",
                 up.reanalyses, up.evaluations, static_cast<long>(unnz), smj.nbElem(), static_cast<void*>(LS));
    if (log)
      DYN::Trace::debug() << DYNLog(MatrixStructureChange) << DYN::Trace::endline;
  }
}

}  // namespace

namespace DYN {

bool
SolverCommon::copySparseToKINSOL(const SparseMatrix& smj, SUNMatrix& JJ, const int& size, sunindextype * lastRowVals) {
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
  if (patternCacheEnabled()) {
    propagateWithUnionPattern(smj, JJ, size, LS, log);
    return;
  }
  bool matrixStructChange = copySparseToKINSOL(smj, JJ, size, *lastRowVals);

  if (matrixStructChange) {
    SUNLinSol_KLUReInit(LS, JJ, SM_NNZ_S(JJ), 2);  // reinit symbolic factorisation
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
