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

#include "solver.h"

namespace g2o {

Solver::Solver(SparseOptimizer* optimizer) :
  _optimizer(optimizer), _x(0), _b(0), _xSize(0), _maxXSize(0),
  _isLevenberg(false)
{
}

Solver::~Solver()
{
  delete[] _x;
  delete[] _b;
}

void Solver::resizeVector(size_t sx)
{
  if (_maxXSize < sx) {
    _maxXSize = 2*sx;
    delete[] _x;
    delete[] _b;
    _x = new double[_maxXSize];
    _b = new double[_maxXSize];
  }
  _xSize = sx;
}

void Solver::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

void Solver::setLevenberg(bool levenberg)
{
  _isLevenberg = levenberg;
}

} // end namespace
