add_executable(
    gjkdemo
    Window.cpp
    Shape.cpp
    main.cpp)
target_link_libraries(gjkdemo PUBLIC engine Qt5::Widgets)
