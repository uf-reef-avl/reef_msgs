project(runTest)
#add_subdirectory(lib/googletest)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(runTest dynamics_test.cpp quat_test.cpp conversion_test.cpp euler_angle_test.cpp axis_angle_test.cpp include/test_utilities.h DCM_test.cpp rodriguez_parameter_test.cpp)
target_link_libraries(runTest gtest gtest_main)
target_link_libraries(runTest reef_msgs)
target_link_libraries(runTest ${GTEST_LIBRARIES})