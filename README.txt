g2o - General Graph Optimization

g2o is licensed under LGPL version 3, whereas the following parts are
licensed under GPL version 3:
- g2o_viewer
- g2o_incremental

See the doc folder for the full text of the licenses.

g2o is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
licenses for more details.


Requirements:
g2o requires cmake to build. The other requirements are optional.
  * cmake             http://www.cmake.org/
  * suitesparse       http://www.cise.ufl.edu/research/sparse/SuiteSparse/
  * Qt4               http://qt.nokia.com/
  * libQGLViewer      http://www.libqglviewer.com/

  On Ubuntu / Debian these dependencies are given by installing the
  following packages.
    - cmake
    - libsuitesparse-dev
    - libqt4-dev
    - qt4-qmake
    - libqglviewer-qt4-dev


Compilation:
Our primary development platform is Linux. Experimental support for
Mac OS X and Windows (with MinGW as a compiler).
We recommend a so-called out of source build which can be achieved
by the following command sequence.

- mkdir build
- cd build
- cmake ../
- make

The binaries will be placed in bin and the libraries in lib which
are both located in the top-level folder.



Acknowledgments:
We thank Simon Julier for submitting patches to achieve compatibility
with Mac OS X and Michael Eriksen for submitting patches to compile
with MSVC.


Contact information:
Rainer Kuemmerle <kuemmerl@informatik.uni-freiburg.de>
Giorgio Grisetti <grisetti@dis.uniroma1.it>
Hauke Strasdat <strasdat@gmail.com>
Kurt Konolige <konolige@willowgarage.com>
Wolfram Burgard <burgard@informatik.uni-freiburg.de>
