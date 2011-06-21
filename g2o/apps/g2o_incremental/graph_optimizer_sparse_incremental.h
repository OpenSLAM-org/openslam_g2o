#ifndef GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H
#define GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/apps/g2o_interactive/graph_optimizer_sparse_online.h"

#include "linear_solver_cholmod_online.h"

namespace g2o {

  class SparseOptimizerIncremental : public SparseOptimizerOnline
  {
    public:
      SparseOptimizerIncremental();
      ~SparseOptimizerIncremental();

      int optimize(int iterations, bool online = false);

      virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);

      virtual bool initSolver(int dimension, int batchEveryN);

    protected:
      SparseBlockMatrix<MatrixXd> _updateMat;
      cholmod_common _cholmodCommon;
      CholmodExt* _cholmodSparse;
      cholmod_factor* _cholmodFactor;
      cholmod_triplet* _permutedUpdate;
      cholmod_factor* _L;

      HyperGraph::VertexSet _touchedVertices;
      Eigen::VectorXi _perm;
      Eigen::VectorXi _cmember;

      bool computeCholeskyUpdate();
  };

}

#endif
