add_executable(
    gjkdemo
    WindowShapeShape.cpp
    WindowShapePoint.cpp
    Shape.cpp
    main.cpp)
target_link_libraries(gjkdemo PUBLIC engine Qt5::Widgets)
