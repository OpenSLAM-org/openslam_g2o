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

#include "types_icp.h"
#include "g2o/core/factory.h"

#include <iostream>

namespace g2o {

  using namespace std;
  using namespace Eigen;
  typedef  Matrix<double, 6, 1> Vector6d;

  void __attribute__ ((constructor)) init_icp_types(void)
  {
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;
    Factory* factory = Factory::instance();
    factory->registerType("VERTEX_XYZN", new HyperGraphElementCreator<VertexPointXYZN>);
    factory->registerType("EDGE_POINT", new HyperGraphElementCreator<EdgePoint3D>);
    factory->registerType("EDGE_V_V_GICP", new HyperGraphElementCreator<Edge_V_V_GICP>);
  }


  VertexPointXYZN::VertexPointXYZN() : BaseVertex<3, Vector6d>()
  {
  }

  bool VertexPointXYZN::read(std::istream& is)
  {
    Vector6d lv;
    for (int i=0; i<6; i++){
      is >> lv[i];
    }
    estimate() = lv;
    return true;
  }

  bool VertexPointXYZN::write(std::ostream& os) const
  {
    Vector6d lv=estimate();
    for (int i=0; i<6; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }


  // 3D constraint
  EdgePoint3D::EdgePoint3D() :
  BaseBinaryEdge<3, EdgeNormal, VertexPointXYZN, VertexCam>()
  {
  }

  bool EdgePoint3D::read(std::istream& is)
  {
    // measured keypoint
    for (int i=0; i<3; i++)
      is >> measurement().pos[i];

    // measured normal
    for (int i=0; i<3; i++)
      is >> measurement().normal[i];

    // don't need this if we don't use it in error calculation (???)
    //    inverseMeasurement() = -measurement();

    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    measurement().makeRot();  // set up rotation matrix

    Matrix3d prec;
    double v = .00001;
    prec << v, 0, 0,
      0, v, 0,
      0, 0, 1;
    Matrix3d &R = measurement().R;
    information() = R.transpose()*prec*R;
    prec.setIdentity();
    //    e.information() = prec;

    //    e.information().setIdentity();

    // [rk] TODO this should go to some other place???
    setRobustKernel(true);
    setHuberWidth(0.01);      // units? m?

    return true;
  }

  bool EdgePoint3D::write(std::ostream& os) const
  {
    for (int i=0; i<3; i++)
      os  << measurement().pos[i] << " ";

    for (int i=0; i<3; i++)
      os  << measurement().normal[i] << " ";

    return os.good();
  }

  // 
  // Rigid 3D constraint between poses, given fixed point offsets
  //

  Edge_V_V_GICP::Edge_V_V_GICP() :
    BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>() {}

  // input two matched points between the frames
  // first point belongs to the first frame, position and normal
  // second point belongs to the second frame, position and normal
  //
  // the measurement variable has type EdgeGICP (see types_icp.h)

  bool Edge_V_V_GICP::read(std::istream& is)
  {
    // measured point and normal
    for (int i=0; i<3; i++)
      is >> measurement().pos0[i];
    for (int i=0; i<3; i++)
      is >> measurement().normal0[i];

    // measured point and normal
    for (int i=0; i<3; i++)
      is >> measurement().pos1[i];
    for (int i=0; i<3; i++)
      is >> measurement().normal1[i];

    // don't need this if we don't use it in error calculation (???)
    //    inverseMeasurement() = -measurement();

    measurement().makeRot();  // set up rotation matrices

    // GICP info matrices

    // point-plane only
    Matrix3d prec;
    double v = .01;
    prec << v, 0, 0,
            0, v, 0,
            0, 0, 1;
    Matrix3d &R = measurement().R0; // plane of the point in vp0
    information() = R.transpose()*prec*R;

    //    information().setIdentity();

    //    setRobustKernel(true);
    setHuberWidth(0.01);      // units? m?

    return true;
  }

  bool Edge_V_V_GICP::write(std::ostream& os) const
  {
    // first point
    for (int i=0; i<3; i++)
      os  << measurement().pos0[i] << " ";
    for (int i=0; i<3; i++)
      os  << measurement().normal0[i] << " ";

    // second point
    for (int i=0; i<3; i++)
      os  << measurement().pos1[i] << " ";
    for (int i=0; i<3; i++)
      os  << measurement().normal1[i] << " ";


    return os.good();
  }




} // end namespace
