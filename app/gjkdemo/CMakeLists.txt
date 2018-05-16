add_executable(
    gjkdemo
    WindowShapeShape.cpp
    WindowShapePoint.cpp
    Shape.cpp
    main.cpp)

target_link_libraries(gjkdemo PUBLIC gjk Qt5::Widgets)

install(
    TARGETS gjkdemo
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
