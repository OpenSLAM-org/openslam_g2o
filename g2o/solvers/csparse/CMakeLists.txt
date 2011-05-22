ADD_LIBRARY(solver_csparse SHARED
  csparse_helper.h         solver_csparse.cpp
  csparse_helper.cpp  linear_solver_csparse.h
)

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

SET_TARGET_PROPERTIES(solver_csparse PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_csparse)

IF(BUILD_CSPARSE)
  TARGET_LINK_LIBRARIES(solver_csparse csparse core)
ELSE()
  TARGET_LINK_LIBRARIES(solver_csparse ${CSPARSE_LIBRARY} core)
ENDIF()


INSTALL(TARGETS solver_csparse
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/solvers/csparse)
