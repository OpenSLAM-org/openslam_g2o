ADD_EXECUTABLE(gicp_demo 
  gicp_demo.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(gicp_demo core solver_cholmod math_groups types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} ${CHOLMOD_LIBRARIES})

