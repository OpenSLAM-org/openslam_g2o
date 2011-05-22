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

#include "vertex_se3_quat.h"
#include "g2o/core/factory.h"

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

#include <iostream>

namespace g2o {

  VertexSE3::VertexSE3() :
    BaseVertex<6, SE3Quat>()
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

#ifdef G2O_HAVE_OPENGL
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
#endif

}
