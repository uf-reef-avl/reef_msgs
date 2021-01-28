//
// Created by paul on 10/1/20.
//

#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"
#include "../../../../../../usr/src/googletest/googlemock/include/gmock/gmock-matchers.h"
#include "../include/reef_msgs/ReefObjectConversion.h"
#include "../include/reef_msgs/ReefMsgsConversionAPI.h"
#include <geometry_msgs/Twist.h>

TEST(test_conversion, test_fromAnyToQuaternion) {
    geometry_msgs::Quaternion v;
    v.x =0;
    v.y = 2;
    v.z = 1;
    v.w = 10;
    reef_msgs::Quaternion b = reef_msgs::fromAnyTypeToQuaternion<geometry_msgs::Quaternion>(v);
//    std::cout << b ;
//    Eigen::Vector4f mat;
//    reef_msgs::fromAnyTypeToQuaternion<Eigen::Vector4f>(mat);
//    reef_msgs::Quaternion a = reef_msgs::Quaternion(0,0.,0.,0.);
}

TEST(test_conversion, test_Quaternion_to_Any) {
    reef_msgs::Quaternion a = reef_msgs::Quaternion(0,0.,5.,0.);
//    std::vector<float> b = reef_msgs::fromQuaternionToAnyType<std::vector<float>>(a);
//    std::vector<double> c = reef_msgs::fromQuaternionToAnyType<std::vector<double>>(a);
//    Eigen::Matrix<double,4,1> d = reef_msgs::fromQuaternionToAnyType<Eigen::Matrix<double,4,1>>(a);
//    Eigen::Matrix<float,4,1> e = reef_msgs::fromQuaternionToAnyType<Eigen::Matrix<float,4,1>>(a);
//    geometry_msgs::Quaternion f = reef_msgs::fromQuaternionToAnyType<geometry_msgs::Quaternion>(a);
//    geometry_msgs::QuaternionStamped g = reef_msgs::fromQuaternionToAnyType<geometry_msgs::QuaternionStamped>(a);
}

TEST(test_conversion, test_Quaternion_to_DCM) {
//    Eigen::Matrix<double,3,3> mat;
//    mat << 1.,1.,1.,
//           0.,0.,0.,
//           1.,0.,1.;
//    reef_msgs::DCM dcm(mat);
//
//    Eigen::Matrix3d DCM = reef_msgs::fromDCMToAnyType<Eigen::Matrix3d>(dcm);
//    std::cout<<DCM;
    geometry_msgs::Quaternion quat;
    quat.x = 0;
    quat.y = 0;
    quat.z = 0;
    quat.w = 0;

    Eigen::Matrix3d DCM = reef_msgs::fromQuaternionToDCM<geometry_msgs::Quaternion,Eigen::Matrix3d>(quat);
//    Eigen::Matrix<double,4,1> d = reef_msgs::fromQuaternionToAnyType<Eigen::Matrix<double,4,1>>(a);
//    Eigen::Matrix<float,4,1> e = reef_msgs::fromQuaternionToAnyType<Eigen::Matrix<float,4,1>>(a);
//    geometry_msgs::Quaternion f = reef_msgs::fromQuaternionToAnyType<geometry_msgs::Quaternion>(a);
//    geometry_msgs::QuaternionStamped g = reef_msgs::fromQuaternionToAnyType<geometry_msgs::QuaternionStamped>(a);
}