INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(tutorial_slam2d
  tutorial_slam2d.cpp
  edge_se2.cpp
  edge_se2_pointxy.cpp
  vertex_point_xy.cpp
  vertex_se2.cpp
)
TARGET_LINK_LIBRARIES(tutorial_slam2d core solver_csparse)
