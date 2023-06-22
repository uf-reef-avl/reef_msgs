//
// Created by paul on 9/29/20.
//


#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/Quaternion.h"
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"
#include "../../../../../../usr/src/googletest/googlemock/include/gmock/gmock-matchers.h"

TEST(test_quat, test_eq) {
    reef_msgs::Quaternion quat = reef_msgs::Quaternion::rand();
    reef_msgs::Quaternion quat1 = reef_msgs::Quaternion::rand();
    std::cout<< quat << std::endl;
    std::cout<< quat1<< std::endl;
    quat = quat1;
    std::cout<< quat<< std::endl;
    std::cout<< quat1<< std::endl;
    quat.normalize();
    std::cout<< quat << std::endl;
    reef_msgs::Quaternion quat2 = reef_msgs::Quaternion::eye();
    std::cout<< quat2<< std::endl;
    quat2.setQuaternion(5.,5.,5.,5.);
    std::cout<< quat2<< std::endl;
    quat2.normalize();
    std::cout<< quat2<< std::endl;
    quat2.imagine();
    std::cout<< quat2<< std::endl;
    reef_msgs::Quaternion quat3(0.4,0.3,0.2,5.);
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
    auto result = reef_msgs::Quaternion::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
    m<< 0.2,0.5,0.6,
            0.4,0.3,0.2,
            -0.2,-0.5,-0.6;
    quatFromDCM << 0.4276029,   0.80769437, -0.1425343,  -0.38009147;
    result = reef_msgs::Quaternion::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
    m<< 0.2,0.5,-0.6,
            0.4,-0.3,0.2,
            -0.2,-0.5,-0.6;
    quatFromDCM << 0.83335958,  0.35715411, -0.31747032,  0.27778653;
    result = reef_msgs::Quaternion::fromDCMtoQuaternion(m);
    ASSERT_NEAR(result(0,0),quatFromDCM(0,0), 0.0001);
    ASSERT_NEAR(result(1,0),quatFromDCM(1,0), 0.0001);
    ASSERT_NEAR(result(2,0),quatFromDCM(2,0), 0.0001);
    ASSERT_NEAR(result(3,0),quatFromDCM(3,0), 0.0001);
}


TEST(test_quat, fromAxisAngle) {
    Eigen::Matrix<double, 3, 1> vector;
    vector << 0.2, 0.4, 0.8;
    Eigen::Matrix<double, 4, 1> quat = reef_msgs::Quaternion::fromAngleAxistoQuaternion(100, vector);
    Eigen::Matrix<double, 4, 1> result;
    result << -0.05725489, -0.11450977, -0.22901955,  0.96496603;
    ASSERT_TRUE(result.isApprox(quat, 0.0001));
}


TEST(test_quat, QuaterniontoOtherMatrixAngle) {
    reef_msgs::Quaternion quat = reef_msgs::Quaternion(0.4, 0.3, 0.2, 5);
    Eigen::Matrix<double, 3, 3> mat = quat.toDCM();
    Eigen::Matrix3d m;
    m << 25.03, 2.24, -2.84,
            -1.76, 24.89, 4.12,
            3.16, -3.88, 24.79;
    ASSERT_TRUE(mat.isApprox(m));
    Eigen::Matrix<double, 3, 3> mat2 = quat.toRotationMatrix();
    m << 25.03, -1.76, 3.16,
            2.24, 24.89, -3.88,
            -2.84, 4.12, 24.79;
    ASSERT_TRUE(mat2.isApprox(m));
}

TEST(test_quat, QuaterniontoOther4DAngle) {

    reef_msgs::Quaternion quat = reef_msgs::Quaternion( 0.2109838268563661, 0.19050591331489203, 0.15409707606385747, 0.9462808319656861);
    Eigen::Matrix<double, 4, 1> mat2 = quat.toAxisAngle();
    Eigen::Matrix<double, 4, 1> vec;
    vec << 0.38798443105430264, 0.3138341762911236, 0.8665888246747364, 0.42968975923188696 ;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
}

TEST(test_quat, QuaterniontoOther3DAngle) {
    reef_msgs::Quaternion quat = reef_msgs::Quaternion(0.4, 0.3, 0.2, -0.3);
    Eigen::Matrix<double, 3, 1> mat2 = quat.toRodriguezParameter();
    Eigen::Matrix<double, 3, 1> vec;
    vec << -1.33333333, -1., -0.66666667 ;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("121");
    vec << 2.80230004, 2.0943951 , 1.62629483;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("123");
    vec << -1.89254688, -0.02000133, -1.24904577;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("131");
    vec << 1.23150371, 2.0943951 , 3.19709116;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("132");
    vec << -1.735945  , -0.36826789, -0.16514868;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("212");
    vec << 1.89254688, 2.26529459, 2.8198421 ;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("213");
    vec << -2.97644398, -0.36826789, 1.735945 ;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("231");
    vec << -1.23150371,  0.12028988, -1.62629483;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("232");
    vec << 3.46334321, 2.26529459, 1.24904577;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("312");
    vec << -1.62629483, -0.12028988, -1.91008894;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("313");
    vec << 3.19709116, 2.40386669, 1.91008894;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("321");
    vec <<  0.78539816, -0.3469169 , -2.35619449;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
    mat2 = quat.toEulerAngle("323");
    vec << 1.62629483, 2.40386669, 3.48088527;
    ASSERT_TRUE(mat2.isApprox(vec, 0.0001));
}

