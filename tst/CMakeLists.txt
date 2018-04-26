add_executable(testGJK testGJK.cpp)
target_link_libraries(testGJK PUBLIC engine Qt5::Test)
add_test(testGJK testGJK)
