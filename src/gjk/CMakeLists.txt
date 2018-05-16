
add_library(gjk SHARED GJK.cpp)

target_link_libraries(gjk PUBLIC Eigen3::Eigen)
target_include_directories(gjk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(
    TARGETS gjk
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
