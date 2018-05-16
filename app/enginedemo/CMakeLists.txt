add_executable(enginedemo main.cpp Preset.cpp)

target_link_libraries(enginedemo PUBLIC engine ui Qt5::Widgets Qt5::Gui)

install(
    TARGETS enginedemo
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
