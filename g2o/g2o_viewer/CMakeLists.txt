INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS base_main_window.ui)
QT4_WRAP_CPP(UI_SOURCES main_window.h)

# ${QT_INCLUDE_DIR} does not appear to work; therefore have to
# manually rig it; also have to add QtCore directly because qglviewer
# doesn't seem to handle the Qt directories properly

INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}/..)

ADD_EXECUTABLE(g2o_viewer
  g2o_qglviewer.cpp
  g2o_viewer.cpp
  primitives.cpp
  main_window.cpp
  stream_redirect.cpp
  gui_sparse_optimizer.cpp
  ${UI_HEADERS}
  ${UI_SOURCES}
)

TARGET_LINK_LIBRARIES(g2o_viewer core g2o_cli_library ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
