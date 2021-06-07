cmake_minimum_required(VERSION 2.8.3)
project(reef_msgs)

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        roscpp
        rospy
        std_msgs
        tf2_eigen
        tf2
)
find_package(Eigen3 REQUIRED)

add_message_files(
        FILES
        DesiredVector.msg
        DesiredState.msg
        DeltaToVel.msg
        XYEstimate.msg
        XYDebugEstimate.msg
        ZEstimate.msg
        ZDebugEstimate.msg
        XYZEstimate.msg
        XYZDebugEstimate.msg
        SyncVerifyEstimates.msg
        SyncEstimateError.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime
        DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS})

add_library(reef_msgs
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
        include/reef_msgs/ReefMsgsConversionAPI.h src/OdomNWU2PoseStampedNEDNode.cpp)

add_dependencies(reef_msgs reef_msgs_generate_messages_cpp)

add_subdirectory(test)

## Specify libraries to link a library or executable target against
target_link_libraries(reef_msgs ${catkin_LIBRARIES})

add_executable(OdomNWU2PoseStampedNED_node
        src/OdomNWU2PoseStampedNEDNode.cpp
        )

target_link_libraries(OdomNWU2PoseStampedNED_node ${catkin_LIBRARIES})
