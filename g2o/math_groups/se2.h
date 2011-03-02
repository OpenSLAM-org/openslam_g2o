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

#ifndef __SE2_H__
#define __SE2_H__

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
  using namespace Eigen;

  class SE2{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SE2():_R(0),_t(0,0){}

    SE2(double x, double y, double theta):_R(theta),_t(x,y){}

    inline const Vector2d& translation() const {return _t;}

    inline Vector2d& translation() {return _t;}

    inline const Rotation2Dd& rotation() const {return _R;}

    inline Rotation2Dd& rotation() {return _R;}
    
    inline SE2 operator * (const SE2& tr2) const{
      SE2 result(*this);
      result._t += _R*tr2._t;
      result._R.angle()+= tr2._R.angle();
      result._R.angle()=normalize_theta(result._R.angle());
      return result;
    }

    inline SE2& operator *= (const SE2& tr2){
      _t+=_R*tr2._t;
      _R.angle()+=tr2._R.angle();
      _R.angle()=normalize_theta(_R.angle());
      return *this;
    }

    inline Vector2d operator * (const Vector2d& v) const {
      return _t+_R*v;
    }

    inline SE2 inverse() const{
      SE2 ret;
      ret._R=_R.inverse();
      ret._R.angle()=normalize_theta(ret._R.angle());
      ret._t=ret._R*(_t*-1.);
      return ret;
    }
    
    inline double operator [](int i) const {
      assert (i>=0 && i<3);
      if (i<2)
	return _t(i);
      return _R.angle();
    }

    inline double& operator [](int i) {
      assert (i>=0 && i<3);
      if (i<2)
	return _t(i);
      return _R.angle();
    }

    inline void fromVector (const Vector3d& v){
      *this=SE2(v[0], v[1], v[2]);
    }

    inline Vector3d toVector() const {
      return Vector3d(_t.x(), _t.y(), _R.angle());
    }

  protected:
    Rotation2Dd _R;
    Vector2d _t;
  };

} // end namespace

#endif
