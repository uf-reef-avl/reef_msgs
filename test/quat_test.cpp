//
// Created by paul on 9/29/20.
//


#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/ReefQuat.h"
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"
#include "../../../../../../usr/src/googletest/googlemock/include/gmock/gmock-matchers.h"


TEST(test_quat, test_eq) {
    reef_msgs::ReefQuat quat = reef_msgs::ReefQuat::rand();
    reef_msgs::ReefQuat quat1 = reef_msgs::ReefQuat::rand();
    std::cout<< quat << std::endl;
    std::cout<< quat1<< std::endl;
    quat = quat1;
    std::cout<< quat<< std::endl;
    std::cout<< quat1<< std::endl;
    quat.normalize();
    std::cout<< quat << std::endl;
    std::cout<<"blblblblblblb"<< std::endl;
    reef_msgs::ReefQuat quat2 = reef_msgs::ReefQuat::eye();
    std::cout<< quat2<< std::endl;
    quat2.setQuaternion(5.,5.,5.,5.);
    std::cout<< quat2<< std::endl;
    quat2.normalize();
    std::cout<<"blblblblblblb"<< std::endl;
    std::cout<< quat2<< std::endl;
    quat2.imagine();
    std::cout<< quat2<< std::endl;
    reef_msgs::ReefQuat quat3(0.4,0.3,0.2,5.);
    quat3.Xi();
    quat3.Psi();
    EXPECT_EQ(1,1);
}

TEST(test_quat, fromDCM) {
    Eigen::Matrix3d m;
    m<< 0.4,0.3,0.2,
        0.4,0.3,0.2,
        0.4,0.3,0.2;
    Eigen::Matrix<double,4,1> quatFromDCM;
    quatFromDCM <<-0.05219958,  0.10439915, -0.05219958 , 0.99179193;
    auto result = reef_msgs::ReefQuat::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
    m<< 0.2,0.5,0.6,
            0.4,0.3,0.2,
            -0.2,-0.5,-0.6;
    quatFromDCM << 0.4276029,   0.80769437, -0.1425343,  -0.38009147;
    result = reef_msgs::ReefQuat::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
    m<< 0.2,0.5,-0.6,
            0.4,-0.3,0.2,
            -0.2,-0.5,-0.6;
    quatFromDCM << 0.83335958,  0.35715411, -0.31747032,  0.27778653;
    result = reef_msgs::ReefQuat::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
}


TEST(test_quat, QuaterniontoAngle) {
    reef_msgs::ReefQuat quat = reef_msgs::ReefQuat(0.4,0.3,0.2,5);
    Eigen::Matrix<double, 3, 3> mat = quat.toDCM();
    Eigen::Matrix3d m;
    m << 25.03,  2.24, -2.84,
        -1.76, 24.89,  4.12,
        3.16, -3.88, 24.79;
    ASSERT_TRUE(mat.isApprox(m));
    Eigen::Matrix<double, 3, 3> mat2 = quat.toRM();
    m << 25.03, -1.76,  3.16,
          2.24 ,24.89, -3.88,
          -2.84,  4.12, 24.79;
    ASSERT_TRUE(mat2.isApprox(m));

//    Eigen::Matrix<double, 3, 3> mat1 = quat.toRotation();
//    mat.normalize();
//    mat1.normalize();
//
//    mat2.normalize();
}

TEST(test_quat, fromAxisAngle) {
    Eigen::Matrix<double, 3, 1> angle;
    angle << 0.2, 0.4, 0.8;
    Eigen::Matrix<double, 4, 1> quat = reef_msgs::ReefQuat::fromAngleAxistoQuaternion(100, angle);
    Eigen::Matrix<double, 4, 1> result;
    result << -0.05725489, -0.11450977, -0.22901955,  0.96496603;
    ASSERT_TRUE(result.isApprox(quat, 0.0001));
}