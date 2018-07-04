add_executable(
    enginedemo
    main.cpp
    WorldReader.h
    WorldReader.cpp)

target_link_libraries(enginedemo PUBLIC engine ui Qt5::Widgets Qt5::Gui jsoncpp)

install(
    TARGETS enginedemo
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
