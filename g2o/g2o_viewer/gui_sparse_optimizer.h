// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GUI_SPARSE_OPTIMIZER_H
#define GUI_SPARSE_OPTIMIZER_H

#include "g2o/core/graph_optimizer_sparse.h"

namespace g2o {

  class G2oQGLViewer;

  /**
   * \brief Optimizer which calls an GUI update after each iteration
   */
  class GuiSparseOptimizer : public SparseOptimizer
  {
    public:
      GuiSparseOptimizer();
      ~GuiSparseOptimizer();

      /**
       * calling updateGL, processEvents to visualize the current state after each iteration
       */
      virtual void postIteration(int);

      G2oQGLViewer* viewer;   ///< the viewer which visualizes the graph
  };

} // end namespace

#endif
