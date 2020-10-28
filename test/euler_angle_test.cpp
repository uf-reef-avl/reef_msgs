//
// Created by paul on 10/27/20.
//
#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/EulerAngle.h"
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"

//if(_eulerTransformation == "121") {
//}else if(_eulerTransformation == "123"){
//}else if(_eulerTransformation == "131"){
//}else if(_eulerTransformation == "132"){
//}else if(_eulerTransformation == "212"){
//}else if(_eulerTransformation == "213"){
//}else if(_eulerTransformation == "231"){
//}else if(_eulerTransformation == "232"){
//}else if(_eulerTransformation == "312"){
//}else if(_eulerTransformation == "313"){
//}else if(_eulerTransformation == "321") {
//}else if(_eulerTransformation == "323"){

TEST(test_euler, eulerAngletoOtherMatrixAngle) {
    reef_msgs::EulerAngle euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"121");
    Eigen::Matrix<double, 3, 3> mat = euler.toDCM();
    Eigen::Matrix3d m;
    m << 0.89165885, 0.07703634, -0.44610526,
   0.34484736, 0.52284883, 0.77955718,
   0.29329984, -0.84893728, 0.43963701;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"123");
    mat = euler.toDCM();
    m << 0.57768677, 0.80054377, -0.15939784, -0.67921535, 0.57974751, 0.45006592, 0.45270796, -0.15173167, 0.87865409;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"131");
    mat = euler.toDCM();
    m << 0.89165885, 0.44610526, 0.07703634, -0.29329984, 0.43963701, 0.84893728, 0.34484736, -0.77955718, 0.52284883;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"132");
    mat = euler.toDCM();
    m << 0.57768677, 0.41864635, -0.70072336, -0.45270796, 0.87865409, 0.15173167, 0.67921535, 0.22956967, 0.69711138;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"212");
    mat = euler.toDCM();
    m << 0.52284883, 0.34484736, -0.77955718, 0.07703634, 0.89165885, 0.44610526, 0.84893728, -0.29329984, 0.43963701;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"213");
    mat = euler.toDCM();
    m << 0.69711138, 0.67921535, 0.22956967, -0.70072336, 0.57768677, 0.41864635, 0.15173167, -0.45270796, 0.87865409;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"231");
    mat = euler.toDCM();
    m << 0.87865409, 0.45270796, -0.15173167, -0.15939784, 0.57768677, 0.80054377, 0.45006592, -0.67921535, 0.57974751;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"232");
    mat = euler.toDCM();
    m << 0.43963701, 0.29329984, -0.84893728, -0.44610526, 0.89165885, 0.07703634, 0.77955718, 0.34484736, 0.52284883;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"312");
    mat = euler.toDCM();
    m << 0.57974751, 0.45006592, -0.67921535, -0.15173167, 0.87865409, 0.45270796, 0.80054377, -0.15939784, 0.57768677;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"313");
    mat = euler.toDCM();
    m << 0.52284883, 0.77955718, 0.34484736, -0.84893728, 0.43963701, 0.29329984, 0.07703634, -0.44610526, 0.89165885;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"321");
    mat = euler.toDCM();
    m << 0.87865409, 0.15173167, -0.45270796, 0.22956967, 0.69711138, 0.67921535, 0.41864635, -0.70072336, 0.57768677;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"323");
    mat = euler.toDCM();
    m << 0.43963701, 0.84893728, -0.29329984, -0.77955718, 0.52284883, 0.34484736, 0.44610526, 0.07703634, 0.89165885;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));


}