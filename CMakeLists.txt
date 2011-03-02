CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(g2o)

# The library prefix
SET(LIB_PREFIX g2o_)

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  #SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# default types and solvers directories are worked out at compilation time
# the enviroment variable G2O_TYPES_DIR and G2O_SOLVER_DIR can be used to override these defaults

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables
IF (NOT EXECUTABLE_OUTPUT_PATH)
  SET(EXECUTABLE_OUTPUT_PATH ${g2o_SOURCE_DIR}/bin)
ENDIF (NOT EXECUTABLE_OUTPUT_PATH)

# Set the output directory for the build libraries
IF (NOT LIBRARY_OUTPUT_PATH)
  SET(LIBRARY_OUTPUT_PATH ${g2o_SOURCE_DIR}/lib)
  IF(WIN32)
    SET(LIBRARY_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH})
  ENDIF(WIN32)
ENDIF(NOT LIBRARY_OUTPUT_PATH)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse and QGLViewer
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${g2o_SOURCE_DIR}/cmake_modules)

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
  SET( BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET( BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  SET(CSPARSE_FOUND TRUE)
ENDIF(CSPARSE_FOUND)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET( G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE})
ENDIF(OPENGL_FOUND)

# For building the GUI
FIND_PACKAGE(QGLViewer)

# Detect OS
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  MESSAGE(STATUS "Compiling on Windows")
ENDIF(WIN32)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  # OS X
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # activate warnings !!!
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")
#MESSAGE("CMAKE_C_FLAGS = ${CMAKE_C_FLAGS}")
#MESSAGE("CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# Set up the top-level include directories
SET( G2O_EIGEN3_INCLUDE ${g2o_SOURCE_DIR}/EXTERNAL/eigen3 CACHE PATH "Directory of Eigen3")
INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${G2O_EIGEN3_INCLUDE})

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL)
ADD_SUBDIRECTORY(g2o)
