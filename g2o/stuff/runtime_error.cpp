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

#include "runtime_error.h"
#include "os_specific.h"
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
using namespace std;

RuntimeError::RuntimeError(const char* fmt, ...) :
  std::exception()
{
  char* auxPtr = NULL;
  va_list arg_list;
  va_start(arg_list, fmt);
  int b = vasprintf(&auxPtr, fmt, arg_list);
  va_end(arg_list);
  if (b > 0)
    _errorMsg = auxPtr;
  else
    _errorMsg = "";
  free(auxPtr);
}

RuntimeError::~RuntimeError() throw()
{
}
