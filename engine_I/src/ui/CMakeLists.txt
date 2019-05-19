
add_library(ui SHARED MainWindow.cpp ViewerWidget.cpp)

target_link_libraries(ui PUBLIC engine Qt5::Core Qt5::Widgets openscenegraph)
target_include_directories(ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(
    TARGETS ui
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
