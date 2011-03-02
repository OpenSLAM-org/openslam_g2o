// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "linear_solver_cholmod.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_CHOLMOD(s, p, l, blockorder) \
  if (1) { \
    std::cerr << "# Using CHOLMOD poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl; \
    LinearSolverCholmod < DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverCholmod<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    linearSolver->setBlockOrdering(blockorder); \
    s = new DIM_TO_SOLVER(p, l)(opt, linearSolver); \
  } else (void)0

namespace g2o {

  static Solver* createSolver(SparseOptimizer* opt, const std::string& solverName)
  {
    g2o::Solver* s = 0;

    if (solverName == "var_cholmod") {
      ALLOC_CHOLMOD(s, -1, -1, false);
    }
    else if (solverName == "fix3_2_cholmod") {
      ALLOC_CHOLMOD(s, 3, 2, true);
    }
    else if (solverName == "fix6_3_cholmod") {
      ALLOC_CHOLMOD(s, 6, 3, true);
    }
    else if (solverName == "fix7_3_cholmod") {
      ALLOC_CHOLMOD(s, 7, 3, true);
    }
    else if (solverName == "fix3_2_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 7, 3, false);
    }

    return s;
  }

  class CholmodSolverCreator : public AbstractSolverCreator
  {
    public:
      CholmodSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual Solver* construct(SparseOptimizer* optimizer)
      {
        return createSolver(optimizer, property().name);
      }
  };

  void __attribute__ ((constructor)) init_solver_cholmod()
  {
    SolverFactory* factory = SolverFactory::instance();
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("var_cholmod", "Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, -1, -1)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix3_2_cholmod", "Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 3, 2)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix6_3_cholmod", "Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix7_3_cholmod", "Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 7, 3)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix3_2_cholmod_scalar", "Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 3, 2)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix6_3_cholmod_scalar", "Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 6, 3)));
    factory->registerSolver(new CholmodSolverCreator(SolverProperty("fix7_3_cholmod_scalar", "Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 7, 3)));
  }

}
