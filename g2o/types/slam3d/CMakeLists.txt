ADD_LIBRARY(types_slam3d SHARED
  vertex_se3_quat.cpp
  vertex_se3_quat.h
  edge_se3_quat.cpp
  edge_se3_quat.h
  types_six_dof_quat.cpp
  types_six_dof_quat.h
)


SET_TARGET_PROPERTIES(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)

TARGET_LINK_LIBRARIES(types_slam3d ${OPENGL_gl_LIBRARY} math_groups core)


INSTALL(TARGETS types_slam3d
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d)


