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

#include <iostream>

namespace g2o {
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

#ifdef G2O_HAVE_OPENGL
    actionLib->registerAction(new VertexSE3DrawAction);
    actionLib->registerAction(new EdgeSE3DrawAction);
#endif
  }

} // end namespace
