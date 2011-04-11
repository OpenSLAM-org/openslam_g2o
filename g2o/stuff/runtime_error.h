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

#ifndef RUNTIME_ERROR_H
#define RUNTIME_ERROR_H

#include <exception>
#include <string>

#include "g2o/stuff/macros.h"

/**
 * \brief a run time error exception
 */
class RuntimeError : public std::exception
{
  public:
    /**
     * constructor which allows to give a error message
     * with printf like syntax
     */
    explicit RuntimeError(const char* fmt, ...) G2O_ATTRIBUTE_FORMAT23;
    virtual ~RuntimeError() throw();
    virtual const char* what() const throw() {return _errorMsg.c_str();}

  protected:
    std::string _errorMsg;
};

#endif
