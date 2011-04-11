#ifndef QGL_GRAPH_VIEWER_H
#define QGL_GRAPH_VIEWER_H

#include "qglviewer.h"

namespace g2o {

  struct SparseOptimizer;

  class Slam2DViewer : public QGLViewer
  {
    public:
      Slam2DViewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
      ~Slam2DViewer();
      virtual void draw();
      void init();

    public:
      SparseOptimizer* graph;
      bool drawCovariance;
  };

} // end namespace

#endif
