ADD_LIBRARY(types_slam2d SHARED
  edge_se2_pointxy_bearing.h  edge_se2_prior.h
  edge_se2.cpp                  edge_se2_pointxy_calib.cpp  types_slam2d.cpp
  edge_se2.h                    edge_se2_pointxy_calib.h    vertex_point_xy.cpp
  edge_se2_multi.cpp            edge_se2_pointxy.cpp        vertex_point_xy.h
  edge_se2_multi.h              edge_se2_pointxy.h          vertex_se2.cpp
  edge_se2_pointxy_bearing.cpp  edge_se2_prior.cpp          vertex_se2.h
)

SET_TARGET_PROPERTIES(types_slam2d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam2d)

TARGET_LINK_LIBRARIES(types_slam2d core ${OPENGL_gl_LIBRARY})

INSTALL(TARGETS types_slam2d
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam2d)
