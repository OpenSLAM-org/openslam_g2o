ADD_LIBRARY(types_slam2d SHARED
  types_slam2d.cpp edge_se2.cpp edge_se2_multi.cpp edge_se2_pointxy_bearing.cpp
  edge_se2_pointxy_calib.cpp edge_se2_pointxy.cpp edge_se2_prior.cpp
  vertex_point_xy.cpp vertex_se2.cpp)

SET_TARGET_PROPERTIES(types_slam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d)

TARGET_LINK_LIBRARIES(types_slam2d core ${OPENGL_gl_LIBRARY})

#INSTALL(TARGETS types_slam2d
#  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
#  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#)

