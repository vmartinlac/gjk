add_executable(testGJK testGJK.cpp)
target_link_libraries(testGJK PUBLIC gjk Qt5::Test)
add_test(testGJK testGJK)

add_executable(testEngine testEngine.cpp)
target_link_libraries(testEngine PUBLIC engine Qt5::Test)
add_test(testEngine testEngine)
