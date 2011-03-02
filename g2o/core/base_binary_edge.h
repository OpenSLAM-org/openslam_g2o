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

#ifndef BASE_BINARY_EDGE_H
#define BASE_BINARY_EDGE_H

#include <iostream>
#include <limits>

#include "optimizable_graph.h"

namespace g2o {

  using namespace Eigen;


  template <int D, typename E, typename VertexXi, typename VertexXj>
  class BaseBinaryEdge : public OptimizableGraph::Edge
  {
    public:

      typedef VertexXi VertexXiType;
      typedef VertexXj VertexXjType;

      static const int Di = VertexXiType::Dimension;
      static const int Dj = VertexXjType::Dimension;

      static const int Dimension = D;
      typedef E Measurement;
      typedef Matrix<double, D, Di> JacobianXiOplusType;
      typedef Matrix<double, D, Dj> JacobianXjOplusType;
      typedef Matrix<double, D, 1> ErrorVector;
      typedef Matrix<double, D, D> InformationType;

      typedef Map<Matrix<double, Di, Dj>, Matrix<double, Di, Dj>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockType;
      typedef Map<Matrix<double, Dj, Di>, Matrix<double, Dj, Di>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockTransposedType;

      BaseBinaryEdge() : OptimizableGraph::Edge(),
      _hessianRowMajor(false),
      _hessian(0, VertexXiType::Dimension, VertexXjType::Dimension), // HACK we map to the null pointer for initializing the Maps
      _hessianTransposed(0, VertexXjType::Dimension, VertexXiType::Dimension)
      {
        _dimension = D;
        resize(2);
      }

      virtual OptimizableGraph::Vertex* createFrom();
      virtual OptimizableGraph::Vertex* createTo();

      virtual double chi2() const 
      {
        return _error.dot(information()*_error);
      }

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
      //! returns the result of the linearization in the manifold space for the node xj
      const JacobianXjOplusType& jacobianOplusXj() const { return _jacobianOplusXj;}

      virtual void constructQuadraticForm() ;

      virtual int rank() const {return _dimension;}

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

      //! information matrix of the constraint
      const InformationType& information() const { return _information;}
      InformationType& information() { return _information;}
      void setInformation(const InformationType& information) { _information = information;}


      //! accessor functions for the measurement represented by the edge
      const Measurement& measurement() const { return _measurement;}
      Measurement& measurement() { return _measurement;}
      void setMeasurement(const Measurement& m) { _measurement = m;}

      /**
       * accessor functions for the inverse measurement, you may use this to cache the
       * inverse measurement in case you need it to compute, for example, the error vector.
       */
      const Measurement& inverseMeasurement() const { return _inverseMeasurement;}
      Measurement& inverseMeasurement() { return _inverseMeasurement;}
      void setInverseMeasurement(const Measurement& im) { _inverseMeasurement = im;}

    protected:

      bool _hessianRowMajor;
      HessianBlockType _hessian;
      HessianBlockTransposedType _hessianTransposed;
      Measurement _measurement;
      Measurement _inverseMeasurement;
      InformationType _information;
      JacobianXiOplusType _jacobianOplusXi;
      JacobianXjOplusType _jacobianOplusXj;

      ErrorVector        _error;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "base_binary_edge.hpp"

} // end namespace g2o

#endif
