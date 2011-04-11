ADD_EXECUTABLE(sba_demo
  sba_demo.cpp
)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(sba_demo core solver_cholmod types_icp types_sba)


