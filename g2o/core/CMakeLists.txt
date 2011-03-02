ADD_LIBRARY(core SHARED
  optimizable_graph.cpp
  estimate_propagator.cpp
  solver.cpp
  graph_optimizer_sparse.cpp
  batch_stats.cpp
  hyper_graph.cpp
  hyper_dijkstra.cpp
  matrix_structure.cpp
  marginal_covariance_cholesky.cpp
  factory.cpp
  hyper_graph_action.cpp
  solver_factory.cpp
)

SET_TARGET_PROPERTIES(core PROPERTIES OUTPUT_NAME ${LIB_PREFIX}core)

TARGET_LINK_LIBRARIES(core stuff)

#INSTALL(TARGETS core
#  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
#  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#)


