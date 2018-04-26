add_executable(tests tests.cpp)
target_link_libraries(tests PUBLIC engine)
add_test(tests tests)
