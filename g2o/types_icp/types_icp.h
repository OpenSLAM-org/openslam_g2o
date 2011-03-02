// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef TYPES_ICP
#define TYPES_ICP

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/math_groups/sbacam.h"
#include "g2o/types_sba/types_sba.h"
#include "g2o/types_slam3d/types_six_dof_quat.h"
#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

  using namespace Eigen;
  using namespace std;
  typedef  Matrix<double, 6, 1> Vector6d;

/**
 * \brief CamVertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 * qw is assumed to be positive, otherwise there is an ambiguity in qx,qy,qz as a rotation
 * [from types_sba.h]
 */


/**
 * \brief Point vertex with normal, XYZ NxNyNz.
 * Uses a 6-vector to hold position and normal
 */
 class VertexPointXYZN : public BaseVertex<3, Vector6d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    VertexPointXYZN();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOrigin() {
      _estimate.fill(0.);
    }

    virtual void oplus(double* update_)
    {

      Vector3d update;
      for (int i=0; i<3; i++)
        update[i]=update_[i];

      _estimate.head(3) += update;
    }


 protected:
};

// 3D (almost) rigid constraint
//    3 values for position wrt frame
//    3 values for normal wrt frame, not used here
// first two args are the measurement type, second two the connection classes
  class EdgePoint3D : public  BaseBinaryEdge<3, EdgeNormal, VertexPointXYZN, VertexCam> 
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePoint3D();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError()
    {
      // from <ViewPoint> to <Point>
      const VertexPointXYZN *point = static_cast<const VertexPointXYZN*>(_vertices[0]);
      const VertexCam *vp = static_cast<const VertexCam*>(_vertices[1]);

      // calculate the relative 3D position of the point
      Vector4d pt;
      pt.head(3) = point->estimate().head(3);
      pt(3) = 1.0;
      const SBACam &nd = vp->estimate();
//      nd.setTransform();  // this should have been done on the update step
      Vector3d p = nd.w2n * pt;
      //      std::cout << std::endl << "VP   " << vp->estimate() << std::endl;
      //      std::cout << "POINT " << pt.transpose() << std::endl;
      //      std::cout << "PROJ  " << p.transpose() << std::endl;
      //      std::cout << "CPROJ " << perr.transpose() << std::endl;
      //      std::cout << "MEAS  " << _measurement.transpose() << std::endl;

      // error, which is backwards from the normal <observed> - <calculated>
      // <_measurement> is the measured projection
      //      Vector3d m = _measurement.head(3);
      //      Vector3d m = _measurement.pos;
      _error = p - _measurement.pos;
    }

    // don't do jacobians yet
    //    virtual void linearizeOplus();

};


//
// GICP-type edges
// Each measurement is between two rigid points on each 6DOF vertex
//


  //
  // class for edges between two points rigidly attached to vertices
  //

  class EdgeGICP
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
    // point positions
    Vector3d pos0, pos1;

    // unit normals
    Vector3d normal0, normal1;

    // rotation matrix for normal
    Matrix3d R0,R1; 

    // initialize an object
    EdgeGICP()
      {
        pos0.setZero();
        pos1.setZero();
        normal0 << 0, 0, 1;
        normal1 << 0, 0, 1;
        makeRot();
      }

    // set up rotation matrix
    void makeRot()
    {
      Vector3d y;
      y << 0, 1, 0;
      R0.row(2) = normal0;
      y = y - normal0(1)*normal0;
      y.normalize();            // need to check if y is close to 0
      R0.row(1) = y;
      R0.row(0) = normal0.cross(R0.row(1));
      //      cout << normal.transpose() << endl;
      //      cout << R0 << endl << endl;
      //      cout << R0*R0.transpose() << endl << endl;

      y << 0, 1, 0;
      R1.row(2) = normal0;
      y = y - normal0(1)*normal0;
      y.normalize();            // need to check if y is close to 0
      R1.row(1) = y;
      R1.row(0) = normal0.cross(R1.row(1));

    }

    // returns a precision matrix for point-plane 
    Matrix3d prec0(double e)
    {
      makeRot();
      Matrix3d prec;
      prec << e, 0, 0,
              0, e, 0,
              0, 0, 1;
      return R0.transpose()*prec*R0;
    }

  };


  // 3D rigid constraint
  //    3 values for position wrt frame
  //    3 values for normal wrt frame, not used here
  // first two args are the measurement type, second two the connection classes
  class Edge_V_V_GICP : public  BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge_V_V_GICP();

    // I/O functions
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError()
    {
      // from <ViewPoint> to <Point>
      const VertexSE3 *vp0 = static_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3 *vp1 = static_cast<const VertexSE3*>(_vertices[1]);

      // get vp1 point into vp0 frame
      Vector3d p1;
      p1 = vp1->estimate().map(measurement().pos1);
      p1 = vp0->estimate().inverse().map(p1);

      //      cout << endl << "Error computation; points are: " << endl;
      //      cout << p0.transpose() << endl;
      //      cout << p1.transpose() << endl;

      // get their difference
      // this is simple Euclidean distance, for now
      _error = p1 - measurement().pos0;
    }

    // don't do jacobians yet
    //    virtual void linearizeOplus();

  };

} // end namespace

#endif // TYPES_ICP
