#include <iostream>

#include "main_window.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solver_csparse/linear_solver_csparse.h"

#include <QApplication>
using namespace std;
using namespace g2o;

int main(int argc, char** argv)
{
  QApplication qapp(argc, argv);

  MainWindow mw;
  mw.show();

  mw.viewer->graph = new SparseOptimizer();

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(mw.viewer->graph, linearSolver);
  mw.viewer->graph->setSolver(solver);

  while (mw.isVisible()) {
    qapp.processEvents();
    usleep(10000);
  }
  return 0;
}
