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

#include "timeutil.h"
#include <iostream>

namespace g2o {

std::string getTimeAsString(time_t time)
{
  std::string dateStr = ctime(&time);
  if (dateStr.size() == 0)
    return "";
  // remove trailing newline
  dateStr.erase(dateStr.size()-1, 1);
  return dateStr;
}

std::string getCurrentTimeAsString()
{
  return getTimeAsString(time(NULL));
} 


ScopeTime::ScopeTime(const char* title) : _title(title), _startTime(get_time()) {}

ScopeTime::~ScopeTime() {
  std::cerr << _title<<" took "<<1000*(get_time()-_startTime)<<"ms.\n";
}

} // end namespace
