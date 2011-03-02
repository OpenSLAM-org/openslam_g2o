// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef BASE_UNARY_EDGE_H
#define BASE_UNARY_EDGE_H

#include <iostream>
#include <cassert>
#include <limits>

#include "optimizable_graph.h"

namespace g2o {

  using namespace Eigen;

  template <int D, typename E, typename VertexXi>
  class BaseUnaryEdge : public OptimizableGraph::Edge
  {
    public:
      static const int Dimension = D;
      typedef E Measurement;
      typedef VertexXi VertexXiType;
      typedef Matrix<double, D, VertexXiType::Dimension> JacobianXiOplusType;
      typedef Matrix<double, D, 1> ErrorVector;
      typedef Matrix<double, D, D> InformationType;

      BaseUnaryEdge() : OptimizableGraph::Edge()
      {
        _dimension = D;
        resize(1);
      }

      virtual double chi2() const { return _error.dot(information()*_error);}

      virtual void robustifyError()
      {
        double nrm = sqrt(_error.dot(information()*_error));
        double w = sqrtOfHuberByNrm(nrm,_huberWidth);
        _error *= w;
      }

      virtual const double* errorData() const { return _error.data();}
      virtual double* errorData() { return _error.data();}
      const ErrorVector& error() const { return _error;}
      ErrorVector& error() { return _error;}

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
       */
      virtual void linearizeOplus();

      //! returns the result of the linearization in the manifold space for the node xi
      const JacobianXiOplusType& jacobianOplusXi() const { return _jacobianOplusXi;}

      virtual void constructQuadraticForm();

      virtual int rank() const {return _dimension;}

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual void mapHessianMemory(double*, int, int, bool) {assert(0 && "BaseUnaryEdge does not map memory of the Hessian");}

      const InformationType& information() const { return _information;}
      InformationType& information() { return _information;}
      void setInformation(const InformationType& information) { _information = information;}

      // accessor functions for the measurement represented by the edge
      const Measurement& measurement() const { return _measurement;}
      Measurement& measurement() { return _measurement;}
      void setMeasurement(const Measurement& m) { _measurement = m;}

      const Measurement& inverseMeasurement() const { return _inverseMeasurement;}
      Measurement& inverseMeasurement() { return _inverseMeasurement;}
      void setInverseMeasurement(const Measurement& im) { _inverseMeasurement = im;}

    protected:

      Measurement _measurement;
      Measurement _inverseMeasurement;
      InformationType _information;
      JacobianXiOplusType _jacobianOplusXi;

      ErrorVector        _error;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_unary_edge.hpp"

} // end namespace g2o

#endif
