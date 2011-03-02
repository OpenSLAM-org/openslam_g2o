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

#include "dl_wrapper.h"

#ifdef UNIX
#include <dlfcn.h>
#include <regex.h>
#include <wordexp.h>
#endif

#include <sys/types.h>
#include <dirent.h>

#include <cstdio>
#include <iostream>
#include <algorithm>

using namespace std;

#ifdef WINDOWS
namespace {

  /*
   * posted by Malcolm McLean on comp.lang.c
   */
  static int chmatch(const char *target, const char *pat);
  /*
     wildcard matcher.
Params: str - the target string
pattern - pattern to match
Returns: 1 if match, 0 if not.
Notes: ? - match any character
   * - match zero or more characters
   [?], [*], escapes,
   [abc], match a, b or c.
   [A-Z] [0-9] [*-x], match range.
   [[] - match '['.
   [][abc] match ], [, a, b or c
   */
  int matchwild(const char *str, const char *pattern)
  {
    const char *target = str;
    const char *pat = pattern;
    int gobble;
    while( (gobble = chmatch(target, pat)) )
    {
      target++;
      pat += gobble;
    }
    if(*target == 0 && *pat == 0)
      return 1;
    else if(*pat == '*')
    {
      while(pat[1] == '*')
        pat++;
      if(pat[1] == 0)
        return 1;
      while(*target)
        if(matchwild(target++, pat+1))
          return 1;
    }
    return 0;
  }

  /*
     match a character.
Parmas: target - target string
pat - pattern string.
Returns: number of pat character matched.
Notes: means that a * in pat will return zero
*/
  static int chmatch(const char *target, const char *pat)
  {
    char *end, *ptr;
    if(*pat == '[' && (end = strchr(pat, ']')) )
    {
      /* treat close bracket following open bracket as character */
      if(end == pat + 1)
      {
        end = strchr(pat+2, ']');
        /* make "[]" with no close mismatch all */
        if(end == 0)
          return 0;
      }
      /* allow [A-Z] and like syntax */
      if(end - pat == 4 && pat[2] == '-' && pat[1] <= pat[3]) {
        if(*target >= pat[1] && *target <= pat[3]) {
          return 5;
        } else {
          return 0;
        }
      }
      /* search for character list contained within brackets */
      ptr = strchr(pat+1, *target);
      if(ptr != 0 && ptr < end)
        return end - pat + 1;
      else
        return 0;
    }
    if(*pat == '?' && *target != 0)
      return 1;
    if(*pat == '*')
      return 0;
    if(*target == 0 || *pat == 0)
      return 0;
    if(*target == *pat)
      return 1;
    return 0;
  }

}
#endif

namespace g2o {

DlWrapper::DlWrapper()
{
}

DlWrapper::~DlWrapper()
{
  clear();
}

int DlWrapper::openLibraries(const std::string& directory, const std::string& pattern)
{
  string expandedDirName = directory;
# ifdef UNIX
  wordexp_t p;
  wordexp(directory.c_str(), &p, 0);
  if (p.we_wordc > 0)
    expandedDirName = p.we_wordv[0];
  wordfree(&p);
# endif

  cerr << "# loading libraries from " << expandedDirName << "\t pattern: " << pattern << endl;
# ifdef UNIX
  regex_t rx;
  regmatch_t* matches = 0;
  if (pattern.size() > 0) {
    int reg_status = regcomp(&rx, pattern.c_str(), REG_EXTENDED);
    if (reg_status != 0) {
      char errorbuf[1024];
      regerror(reg_status , &rx, errorbuf, sizeof(errorbuf));
      fprintf(stderr, "Invalid regular expression: %s\n", errorbuf);
      regfree(&rx);
      return -1;
    }
    matches = new regmatch_t[rx.re_nsub + 1];
  }
# endif

  int numLibs = 0;
  DIR* dir = opendir(expandedDirName.c_str());
  if (dir) {
    struct dirent* d = 0;
    while (1) {
      d = readdir(dir);
      if (! d)
        break;

      string filename = expandedDirName + "/" + d->d_name;
      //cerr << "filename " << filename << endl;
      if (pattern.size() > 0) {
#       ifdef UNIX
        int reg_status = regexec(&rx, filename.c_str(), rx.re_nsub + 1, matches, 0);
        if (reg_status != 0)
          continue;
#       elif defined (WINDOWS)
        int match = matchwild(filename.c_str(), pattern.c_str());
        if (! match)
          continue;
#       endif
      }

      if (find(_filenames.begin(), _filenames.end(), filename) != _filenames.end())
        continue;

      // open the lib
      //cerr << "loading " << filename << endl;
      if (openLibrary(filename))
        numLibs++;

    }
    closedir(dir);
  }

# ifdef UNIX
  if (pattern.size() > 0) {
    delete[] matches;
    regfree(&rx);
  }
# endif

  return numLibs;
}

void DlWrapper::clear()
{
#ifdef UNIX
  for (size_t i = 0; i < _handles.size(); ++i) {
    dlclose(_handles[i]);
  }
#elif defined(WINDOWS)
  for (size_t i = 0; i < _handles.size(); ++i) {
    FreeLibrary(_handles[i]);
  }
#endif
  _filenames.clear();
  _handles.clear();
}

bool DlWrapper::openLibrary(const std::string& filename)
{
# ifdef UNIX
  void* handle = dlopen(filename.c_str(), RTLD_LAZY);
  if (! handle) {
    cerr << "Cannot open library: " << dlerror() << '\n';
    return false;
  }
# elif defined (WINDOWS)
  HMODULE handle = LoadLibrary(filename.c_str());
  if (! handle) {
    cerr << "Cannot open library." << endl;
    return false;
  }
# endif

  //cerr << "loaded " << filename << endl;

  _filenames.push_back(filename);
  _handles.push_back(handle);
  return true;
}

} // end namespace g2o

