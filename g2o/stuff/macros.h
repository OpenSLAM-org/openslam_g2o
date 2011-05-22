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

#include <limits>
#include <cmath>

#ifndef G2O_MACROS_H
#define G2O_MACROS_H

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

#if defined(__GNUC__)
#  define G2O_ATTRIBUTE_CONSTRUCTOR __attribute__ ((constructor))
#  define G2O_ATTRIBUTE_UNUSED __attribute__((unused))
#  define G2O_ATTRIBUTE_FORMAT12 __attribute__ ((format (printf, 1, 2)))
#  define G2O_ATTRIBUTE_FORMAT23 __attribute__ ((format (printf, 2, 3)))
#else
#  ifndef __PRETTY_FUNCTION__
#    define __PRETTY_FUNCTION__ ""
#  endif
#  define G2O_ATTRIBUTE_CONSTRUCTOR
#  define G2O_ATTRIBUTE_UNUSED
#  define G2O_ATTRIBUTE_FORMAT12
#  define G2O_ATTRIBUTE_FORMAT23
#endif

inline bool g2o_isnan(double x)
{
#if defined(__GNUC__)
  return std::isnan(x);
#else
  return x != x;
#endif
}

// some macros that are only useful for c++
#ifdef __cplusplus

#define G2O_FSKIP_LINE(f) \
   {char c=' ';while(c != '\n' && f.good() && !(f).eof()) (f).get(c);}

#ifndef PVAR
  #define PVAR(s) \
    #s << " = " << (s) << std::flush
#endif

#ifndef FIXED
#define FIXED(s) \
  std::fixed << s << std::resetiosflags(std::ios_base::fixed)
#endif

#endif // __cplusplus

#endif
