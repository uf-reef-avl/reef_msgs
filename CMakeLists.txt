cmake_minimum_required(VERSION 3.5)
project(reef_msgs)

set(LIB_NAME lib_reef_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
   set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
   set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
   find_package(ament_lint_auto REQUIRED)
   # the following line skips the linter which checks for copyrights
   # uncomment the line when a copyright and license is not present in all source files
   #set(ament_cmake_copyright_FOUND TRUE)
   # the following line skips cpplint (only works in a git repo)
   # uncomment the line when this package is not in a git repo
   #set(ament_cmake_cpplint_FOUND TRUE)
   ament_lint_auto_find_test_dependencies()
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DesiredVector.msg"
        "msg/DesiredState.msg"
        "msg/DeltaToVel.msg"
        "msg/XYEstimate.msg"
        "msg/XYDebugEstimate.msg"
        "msg/ZEstimate.msg"
        "msg/ZDebugEstimate.msg"
        "msg/XYZEstimate.msg"
        "msg/XYZDebugEstimate.msg"
        "msg/SyncVerifyEstimates.msg"
        "msg/SyncEstimateError.msg"
        "srv/GpsPosition.srv"
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


include_directories(${EIGEN_INCLUDE_DIRS})

add_library(${LIB_NAME} SHARED
        src/dynamics.cpp
        include/reef_msgs/dynamics.h
        include/reef_msgs/matrix_operation.h
        src/matrix_operation.cpp
        include/reef_msgs/AngleRepresentationInterface.h
        src/AngleRepresentationInterface.cpp
        include/reef_msgs/ReefObjectConversion.h
        src/Quaternion.cpp
        include/reef_msgs/Quaternion.h
        src/EulerAngle.cpp include/reef_msgs/EulerAngle.h
        src/RodriguezParameter.cpp
        include/reef_msgs/RodriguezParameter.h
        src/AxisAngle.cpp
        include/reef_msgs/AxisAngle.h
        src/DCM.cpp include/reef_msgs/DCM.h
        src/RotationMatrix.cpp
        include/reef_msgs/RotationMatrix.h
        include/reef_msgs/ReefMsgsConversionAPI.h)

ament_target_dependencies(${LIB_NAME} rclcpp tf2 Eigen3)
ament_export_targets(${LIB_NAME} HAS_LIBRARY_TARGET)


install(
        TARGETS ${LIB_NAME}
        EXPORT ${LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)


add_executable(OdomNWU2PoseStampedNED_node
        src/OdomNWU2PoseStampedNEDNode.cpp
        )
ament_target_dependencies(OdomNWU2PoseStampedNED_node rclcpp geometry_msgs nav_msgs)

install(TARGETS
        OdomNWU2PoseStampedNED_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_export_dependencies(rosidl_default_runtime)
 
ament_package()

#install(DIRECTORY
#        launch
#        DESTINATION share/${PROJECT_NAME}
#        )

