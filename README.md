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
    Eigen::Matrix<double,3,1>  euler = reef_msgs::fromQuaternionToEulerAngle<geometry_msgs::Quaternion, Eigen::Matrix<double,3,1>>(quat1);

There is 6 angle representation supported right now : 

    -Quaternion

    -AxisAngle

    -DCM

    -EulerAngle: is in (yaw, pich, roll); example:

        std::vector inputvec = {yaw,pitch,roll};
        
        geometry_msgs::Quaternion quat = fromEulerAngleToQuaternion<std::vector,geometry_msgs::Quaternion>(inputvec, "321") -> 321 can be changed to any euler angle application

    -RotationMatrix

    -Rodriguez Parameter


You can use the following syntax in C++ to convert them:

    output type object = reef_msgs::from***Input angle***To***Ouput Angle***<***input type***, ***output type***>(input_object);

Not all the object type are supported for now, you can use std::vector, Eigen Matrix or some ros messages. To implement new object type conversion, you will have to modify the file ***ReefObjectConversion.h*** and add some static conversion.





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
