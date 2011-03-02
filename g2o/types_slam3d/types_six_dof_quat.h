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

#ifndef THREE_D_TYPES
#define THREE_D_TYPES

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

#define THREE_D_TYPES_ANALYTIC_JACOBIAN

namespace g2o {

  using namespace Eigen;

/**
 * \brief 3D pose Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
class VertexSE3 : public BaseVertex<6, SE3Quat>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSE3();

    virtual void setToOrigin() {
      _estimate = SE3Quat() ;
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void oplus(double* update)
    {
      Map<Vector6d> v(update);
      SE3Quat increment(v);
      _estimate *= increment;
    }
};

  class VertexSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
						HyperGraphElementAction::Parameters* params_ );
  };

  class VertexSE3DrawAction: public DrawAction{
  public:
    VertexSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
						HyperGraphElementAction::Parameters* params_ );
  };


/**
 * \brief 3D edge between two VertexSE3
 */
 class EdgeSE3 : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3();
    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
      SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error.head<3>() = delta.translation();
      // The analytic Jacobians assume the error in this special form (w beeing positive)
      if (delta.rotation().w() < 0.)
        _error.tail<3>() =  - delta.rotation().vec();
      else
        _error.tail<3>() =  delta.rotation().vec();
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

#ifdef THREE_D_TYPES_ANALYTIC_JACOBIAN
    virtual void linearizeOplus();
#endif
};

  class EdgeSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
						HyperGraphElementAction::Parameters* params_);
  };

  class EdgeSE3DrawAction: public DrawAction{
  public:
    EdgeSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
						HyperGraphElementAction::Parameters* params_);
  };

} // end namespace

#endif
