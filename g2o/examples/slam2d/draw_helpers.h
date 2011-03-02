#ifndef DRAW_HELPERS_H
#define DRAW_HELPERS_H

#include <qgl.h>

namespace g2o {

  /**
   * draw a disk
   */
  void drawDisk(GLfloat radius);

  /**
   * draw a circle using GL_LINES
   */
  void drawCircle(GLfloat radius, int segments = 32);

}

#endif
