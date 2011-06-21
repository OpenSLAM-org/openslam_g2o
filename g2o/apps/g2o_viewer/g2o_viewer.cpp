#include <iostream>

#include "main_window.h"
#include "stream_redirect.h"

#include "gui_hyper_graph_action.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"

#include "g2o/stuff/command_args.h"

#include <QApplication>
using namespace std;
using namespace g2o;

int main(int argc, char** argv)
{
  QApplication qapp(argc, argv);

  string dummy;
  string inputFilename;
  CommandArgs arg;
  arg.param("solverlib", dummy, "", "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "", "specify a types library which will be loaded");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);

  // loading the standard solver /  types
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);

  MainWindow mw;
  mw.updateDisplayedSolvers();
  mw.show();

  // redirect the output that normally goes to cerr to the textedit in the viewer
  StreamRedirect redirect(cerr, mw.plainTextEdit);

  // setting up the optimizer
  SparseOptimizer* optimizer = new SparseOptimizer();
  mw.viewer->graph = optimizer;

  // set up the GUI action
  GuiHyperGraphAction guiHyperGraphAction;
  guiHyperGraphAction.viewer = mw.viewer;
  optimizer->addPostIterationAction(&guiHyperGraphAction);

  if (inputFilename.size() > 0) {
    mw.loadFromFile(QString::fromStdString(inputFilename));
  }

  while (mw.isVisible()) {
    guiHyperGraphAction.dumpScreenshots = mw.actionDump_Images->isChecked();
    qapp.processEvents();
    usleep(10000);
  }

  delete optimizer;
  return 0;
}
