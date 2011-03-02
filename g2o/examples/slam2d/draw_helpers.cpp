#include "draw_helpers.h"

#include <cmath>

namespace g2o {

  /**
   * \brief handle the GLU quadratic
   */
  namespace {
    class GLUWrapper
    {
      public:
        static GLUquadricObj* getQuadradic()
        {
          static GLUWrapper inst;
          return inst._quadratic;
        }
      protected:
        GLUWrapper()
        {
          _quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
          gluQuadricNormals(_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )
        }
        ~GLUWrapper()
        {
          gluDeleteQuadric(_quadratic);
        }
        GLUquadricObj* _quadratic;;
    };
  }

  void drawDisk(GLfloat radius)
  {
    gluDisk(GLUWrapper::getQuadradic(), 0, radius, 32, 1);
  }

  void drawCircle(GLfloat radius, int segments)
  {
    double angleStep = (2 * M_PI / (segments));
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; i++) {
      double angle = i * angleStep;
      float x = radius * cos(angle);
      float y = radius * sin(angle);
      glVertex3f(x, y, 0.f);
    }
    glEnd();
  }

} // end namespace g2o
