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

#include "g2o_common.h"

#include "dl_wrapper.h"
#include "g2o/stuff/string_tools.h"

#include <vector>
#include <iostream>
#include <cstdlib>
using namespace std;

/*
 * setting up the library filename patterns for the different OS
 */
#ifdef __APPLE__
#define SO_EXT "dylib"
#elif defined (WINDOWS)
#define SO_EXT ".dll"
#else // Linux
#define SO_EXT ".so"
#endif

#ifdef UNIX
#  define TYPES_PATTERN ".*_types_.*" SO_EXT "$"
#  define SOLVER_PATTERN ".*_solver_.*" SO_EXT "$"
#elif defined (WINDOWS)
#  define TYPES_PATTERN "*_types_*" SO_EXT
#  define SOLVER_PATTERN "*_solver_*" SO_EXT
#endif

namespace g2o {

void findArguments(const std::string& option, vector<string>& args, int argc, char** argv)
{
  args.clear();
  for (int i = 0; i < argc; ++i) {
    if (argv[i] == option && i + 1 < argc) {
      args.push_back(argv[i+1]);
    }
  }
}

void loadStandardTypes(DlWrapper& dlWrapper, int argc, char** argv)
{
  char * envTypesPath = getenv("G2O_TYPES_DIR");
  if (envTypesPath != NULL) {
    vector<string> paths = strSplit(envTypesPath, ":");
    for (vector<string>::const_iterator it = paths.begin(); it != paths.end(); ++it) {
      if (it->size() > 0)
        dlWrapper.openLibraries(*it, TYPES_PATTERN);
    }
  } else {
    dlWrapper.openLibraries(G2O_DEFAULT_TYPES_DIR_, TYPES_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != 0)
    findArguments("-typeslib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end(); ++it) {
    cerr << "Loading types " << *it << endl;
    dlWrapper.openLibrary(*it);
  }
}

void loadStandardSolver(DlWrapper& dlSolverWrapper, int argc, char** argv)
{
  char * envSolversPath = getenv("G2O_SOLVER_DIR");
  if (envSolversPath != NULL) {
    vector<string> paths = strSplit(envSolversPath, ":");
    for (vector<string>::const_iterator it = paths.begin(); it != paths.end(); ++it) {
      if (it->size() > 0)
        dlSolverWrapper.openLibraries(*it, SOLVER_PATTERN);
    }
  } else {
    dlSolverWrapper.openLibraries(G2O_DEFAULT_SOLVER_DIR_, SOLVER_PATTERN);
  }

  vector<string> libs;
  if (argc > 0 && argv != 0)
    findArguments("-solverlib", libs, argc, argv);
  for (vector<string>::const_iterator it = libs.begin(); it != libs.end(); ++it) {
    cerr << "Loading solver " << *it << endl;
    dlSolverWrapper.openLibrary(*it);
  }
}

} // end namespace
