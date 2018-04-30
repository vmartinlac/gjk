
add_library(ui MainWindow.cpp ViewerWidget.cpp)

target_link_libraries(ui PUBLIC engine Qt5::Core Qt5::Widgets ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(ui PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})
