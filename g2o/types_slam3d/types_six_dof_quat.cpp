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

#include "types_six_dof_quat.h"
#include "g2o/core/factory.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <iostream>
namespace g2o {

  // forward declaration for the analytic jacobian
  void  jacobian_3d_qman ( Matrix< double, 6 , 6> &  Ji , Matrix< double, 6 , 6> &  Jj, const double&  z11 , const double&  z12 , const double&  z13 , const double&  z14 , const double&  z21 , const double&  z22 , const double&  z23 , const double&  z24 , const double&  z31 , const double&  z32 , const double&  z33 , const double&  z34 , const double&  xab11 , const double&  xab12 , const double&  xab13 , const double&  xab14 , const double&  xab21 , const double&  xab22 , const double&  xab23 , const double&  xab24 , const double&  xab31 , const double&  xab32 , const double&  xab33 , const double&  xab34 );

  using namespace std;

  void __attribute__ ((constructor)) init_three_d_types(void)
  {
    Factory* factory = Factory::instance();
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;
    factory->registerType("VERTEX_SE3:QUAT", new HyperGraphElementCreator<VertexSE3>);
    factory->registerType("EDGE_SE3:QUAT", new HyperGraphElementCreator<EdgeSE3>);

    HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
    actionLib->registerAction(new VertexSE3WriteGnuplotAction);
    actionLib->registerAction(new EdgeSE3WriteGnuplotAction);
    actionLib->registerAction(new VertexSE3DrawAction);
    actionLib->registerAction(new EdgeSE3DrawAction);
  }

  VertexSE3::VertexSE3() :
    BaseVertex<6, SE3Quat>()
  {
  }

  EdgeSE3::EdgeSE3() :
    BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>()
  {
  }

  bool VertexSE3::read(std::istream& is)
  {
    for (int i=0; i<7; i++)
      is  >> estimate()[i];
    estimate().rotation().normalize();
    return true;
  }

  bool VertexSE3::write(std::ostream& os) const
  {
    for (int i=0; i<7; i++)
      os << estimate()[i] << " ";
    return os.good();
  }

  bool EdgeSE3::read(std::istream& is)
  {
    for (int i=0; i<7; i++)
      is >> measurement()[i];
    measurement().rotation().normalize();
    inverseMeasurement() = measurement().inverse();

    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
    return true;
  }

  bool EdgeSE3::write(std::ostream& os) const
  {
    for (int i=0; i<7; i++)
      os << measurement()[i] << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  information()(i,j);
      }
    return os.good();
  }

  void EdgeSE3::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* to = static_cast<VertexSE3*>(_vertices[1]);
    if (from_.count(from) > 0)
      to->estimate() = from->estimate() * _measurement;
    else
      from->estimate() = to->estimate() * _inverseMeasurement;
  }

#ifdef THREE_D_TYPES_ANALYTIC_JACOBIAN
  void EdgeSE3::linearizeOplus(){
    VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* to = static_cast<VertexSE3*>(_vertices[1]);
    
    Matrix3d izR        = inverseMeasurement().rotation().toRotationMatrix();
    const Vector3d& izt = inverseMeasurement().translation();
    
    SE3Quat iXiXj         = from->estimate().inverse() * to->estimate();
    Matrix3d iRiRj        = iXiXj.rotation().toRotationMatrix();
    const Vector3d& ititj = iXiXj.translation();

    jacobian_3d_qman ( _jacobianOplusXi , _jacobianOplusXj,
		       izR(0,0), izR(0,1), izR(0,2), izt(0),
		       izR(1,0), izR(1,1), izR(1,2), izt(1),
		       izR(2,0), izR(2,1), izR(2,2), izt(2),
		       iRiRj(0,0), iRiRj(0,1), iRiRj(0,2), ititj(0),
		       iRiRj(1,0), iRiRj(1,1), iRiRj(1,2), ititj(1),
		       iRiRj(2,0), iRiRj(2,1), iRiRj(2,2), ititj(2));
  }
#endif


  VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE3).name()){}

  HyperGraphElementAction* VertexSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }
    
    VertexSE3* v =  static_cast<VertexSE3*>(element);
    *(params->os) << v->estimate().translation().x() << " " 
		  << v->estimate().translation().y() << " " 
		  << v->estimate().translation().z() << " ";
    *(params->os) << v->estimate().rotation().x() << " " 
		  << v->estimate().rotation().y() << " " 
		  << v->estimate().rotation().z() << " " << std::endl;
    return this;
  }

  VertexSE3DrawAction::VertexSE3DrawAction(): DrawAction(typeid(VertexSE3).name()){}

  HyperGraphElementAction* VertexSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							   HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    VertexSE3* that = static_cast<VertexSE3*>(element);
    float xSize=1;
    float ySize2=.5;
    float zSize2=.5;

    static bool first = true;
    static Vector3f p[6];
    if (first) {
      first = false;
      p[0] << xSize, 0., 0.;
      p[1] << -xSize, -ySize2, -zSize2;
      p[2] << -xSize,  ySize2, -zSize2;
      p[3] << -xSize,  ySize2,  zSize2;
      p[4] << -xSize, -ySize2,  zSize2;
      p[5] << -xSize, -ySize2, -zSize2;
    }

    glColor3f(0.5,0.5,0.8);
    glPushMatrix();
    glTranslatef(that->estimate().translation().x(),that->estimate().translation().y(),that->estimate().translation().z());
    AngleAxisd aa(that->estimate().rotation());
    glRotatef(RAD2DEG(aa.angle()),aa.axis().x(),aa.axis().y(),aa.axis().z());

    glBegin(GL_TRIANGLES);
    for (int i = 1; i < 5; ++i) {
      Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
      glNormal3f(normal.x(), normal.y(), normal.z());
      glVertex3f(p[0].x(), p[0].y(), p[0].z());
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
      glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
    }
  
    glNormal3f(-1,0,0);
    for (int i = 1; i < 4; ++i)
      glVertex3f(p[i].x(), p[i].y(), p[i].z());
    for (int i = 3; i < 6; ++i)
      glVertex3f(p[i].x(), p[i].y(), p[i].z());

    glEnd();
    glPopMatrix();
    return this;
  }

  EdgeSE3WriteGnuplotAction::EdgeSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertices()[1]);
    *(params->os) << fromEdge->estimate().translation().x() << " " 
		  << fromEdge->estimate().translation().y() << " " 
		  << fromEdge->estimate().translation().z() << " ";
    *(params->os) << fromEdge->estimate().rotation().x() << " " 
		  << fromEdge->estimate().rotation().y() << " " 
		  << fromEdge->estimate().rotation().z() << " " << std::endl;
    *(params->os) << toEdge->estimate().translation().x() << " " 
		  << toEdge->estimate().translation().y() << " " 
		  << toEdge->estimate().translation().z() << " ";
    *(params->os) << toEdge->estimate().rotation().x() << " " 
		  << toEdge->estimate().rotation().y() << " " 
		  << toEdge->estimate().rotation().z() << " " << std::endl;
    *(params->os) << std::endl;
    return this;
  }

  EdgeSE3DrawAction::EdgeSE3DrawAction(): DrawAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							 HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertices()[1]);
    glColor3f(0.5,0.5,0.8);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),fromEdge->estimate().translation().z());
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }

} // end namespace
