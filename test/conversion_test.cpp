//
// Created by paul on 10/1/20.
//

#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/ReefQuat.h"
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"
#include "../../../../../../usr/src/googletest/googlemock/include/gmock/gmock-matchers.h"
#include "../include/reef_msgs/ReefObjectConversion.h"
#include <geometry_msgs/Twist.h>

TEST(test_conversion, test_fromAnyToReefQuat) {
    geometry_msgs::Quaternion v;
    reef_msgs::fromAnyTypeToReefQuat<geometry_msgs::Quaternion>(v);
    Eigen::Matrix<double,3, 1> mat;
    reef_msgs::fromAnyTypeToReefQuat<Eigen::Matrix<double,3, 1>>(mat);
    reef_msgs::ReefQuat a = reef_msgs::ReefQuat(0,0.,0.,0.);
    reef_msgs::fromReefQuatToAnyType<geometry_msgs::Twist>(a);
}