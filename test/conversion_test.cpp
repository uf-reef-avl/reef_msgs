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
    v.x =0;
    v.y = 2;
    v.z = 1;
    v.w = 10;
    reef_msgs::ReefQuat b = reef_msgs::fromAnyTypeToReefQuat<geometry_msgs::Quaternion>(v);
    std::cout << b ;
    Eigen::Vector4f mat;
    reef_msgs::fromAnyTypeToReefQuat<Eigen::Vector4f>(mat);
    reef_msgs::ReefQuat a = reef_msgs::ReefQuat(0,0.,0.,0.);
}

TEST(test_conversion, test_ReefQuat_to_Any) {
    reef_msgs::ReefQuat a = reef_msgs::ReefQuat(0,0.,5.,0.);
    std::vector<float> b = reef_msgs::fromReefQuatToAnyType<std::vector<float>>(a);
    std::vector<double> c = reef_msgs::fromReefQuatToAnyType<std::vector<double>>(a);
    Eigen::Matrix<double,4,1> d = reef_msgs::fromReefQuatToAnyType<Eigen::Matrix<double,4,1>>(a);
    Eigen::Matrix<float,4,1> e = reef_msgs::fromReefQuatToAnyType<Eigen::Matrix<float,4,1>>(a);
    geometry_msgs::Quaternion f = reef_msgs::fromReefQuatToAnyType<geometry_msgs::Quaternion>(a);
    geometry_msgs::QuaternionStamped g = reef_msgs::fromReefQuatToAnyType<geometry_msgs::QuaternionStamped>(a);
}