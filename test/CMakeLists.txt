include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(runTest dynamics_test.cpp quat_test.cpp conversion_test.cpp)

target_link_libraries(runTest gtest gtest_main)
target_link_libraries(runTest reef_msgs)