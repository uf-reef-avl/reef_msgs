#REEF_MSGS

This package contains various helper functions for attitude representation and ROS Support. 

## Angle representation conversion

**Reef_msgs** has an API  which helps angle manipulation. It permits in one line to transform any angle representation between: axis angle, euler angle, rotation matrix, direction cosine matrix, quaternion; represented as any C++ type (geometry_msgs, vector, eigen matrix ...) to another angle representation in any other C++ type.

Here are some examples how to use it:

    #include <reef_msgs/ReefMsgsConversionAPI.h>
    geometry_msgs::Quaternion quat;
    quat.x = 0;
    quat.y = 0;
    quat.z = 0;
    quat.w = 1;
    Eigen::Matrix3d DCM = reef_msgs::fromQuaternionToDCM<geometry_msgs::Quaternion,Eigen::Matrix3d>(quat); // fullfill the template with the input C++ type and the output C++ type
    geometry_msgs::Quaternion quat1 = reef_msgs::fromDCMToQuaternion<Eigen::Matrix3d,geometry_msgs::Quaternion>(DCM);

## Functions helper

It adds some useful functions to manipulate matrices and permits loading them as ros parameters.

## Node helper

Reef_msgs features also a node to transform simulation odometry msgs in NWU to PoseStamped msgs in NED, Use it with a launch file like this::

        <launch>
            <node name="odomNWU2poseStampedNED" pkg="reef_msgs" type="OdomNWU2PoseStampedNED_node" clear_params="true" output = "screen">
            <remap from="odom_nwu" to="*** input odom NWU topic *** "/>
            <remap from="pose_stamped_ned" to="*** output posestamped NED topic ***"/>
            </node>

        </launch>