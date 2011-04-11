ADD_LIBRARY(core SHARED
base_edge.h
base_binary_edge.h          hyper_graph_action.cpp
base_binary_edge.hpp        hyper_graph_action.h
base_multi_edge.h           hyper_graph.cpp
base_multi_edge.hpp         hyper_graph.h
base_unary_edge.h           linear_solver.h
base_unary_edge.hpp         marginal_covariance_cholesky.cpp
base_vertex.h               marginal_covariance_cholesky.h
base_vertex.hpp             matrix_structure.cpp
batch_stats.cpp             matrix_structure.h
batch_stats.h               openmp_mutex.h
block_solver.h              optimizable_graph.cpp
block_solver.hpp            optimizable_graph.h
solver.cpp
creators.h                  solver_factory.cpp
estimate_propagator.cpp     solver_factory.h
estimate_propagator.h       solver.h
factory.cpp                 solver_property.h
factory.h                   sparse_block_matrix.h
graph_optimizer_sparse.cpp  sparse_block_matrix.hpp
graph_optimizer_sparse.h
hyper_dijkstra.cpp          structure_only_solver.h
hyper_dijkstra.h
)

SET_TARGET_PROPERTIES(core PROPERTIES OUTPUT_NAME ${LIB_PREFIX}core)

TARGET_LINK_LIBRARIES(core stuff)

INSTALL(TARGETS core
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/core)
