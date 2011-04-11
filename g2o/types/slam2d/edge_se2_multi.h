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

#ifndef EDGE_SE2_MULTI_H
#define EDGE_SE2_MULTI_H

#include "vertex_se2.h"
#include "g2o/core/base_multi_edge.h"

namespace g2o {

  /**
   * \brief Pairwise SE2 Edge as MultiEdge. Mainly for debugging.
   */
  class EdgeSE2Multi: public BaseMultiEdge<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2Multi();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
        SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
        _error = delta.toVector();
      }
  };

}

#endif
