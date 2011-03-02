PROJECT(stuff)

ADD_LIBRARY(stuff SHARED
  timeutil.cpp
  filesys_tools.cpp
  string_tools.cpp
  runtime_error.cpp
  os_specific.c
  command_args.cpp
  sparse_helper.cpp
)

SET_PROPERTY(TARGET stuff APPEND PROPERTY COMPILE_DEFINITIONS FSLINE)

SET_TARGET_PROPERTIES(stuff PROPERTIES OUTPUT_NAME ${LIB_PREFIX}stuff)

#INSTALL(TARGETS stuff
#  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
#  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#)


