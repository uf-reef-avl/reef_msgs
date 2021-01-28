//
// Created by paul on 10/27/20.
//
#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include "../include/reef_msgs/EulerAngle.h"
#include "../include/reef_msgs/dynamics.h"
#include "../include/reef_msgs/matrix_operation.h"
#include "./include/test_utilities.h"
#include <fstream>

TEST(test_euler, eulerAngletoOtherMatrixAngle) {
    reef_msgs::EulerAngle euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"121");
    Eigen::Matrix<double, 3, 3> mat = euler.toDCM();
    Eigen::Matrix<double, 3, 3> matRM = euler.toRotationMatrix();
    Eigen::Matrix3d m;
    m << 0.89165885, 0.07703634, -0.44610526,
   0.34484736, 0.52284883, 0.77955718,
   0.29329984, -0.84893728, 0.43963701;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    m << 0.89165885,0.34484736, 0.29329984,
            0.07703634, 0.52284883, -0.84893728,
            -0.44610526,0.77955718, 0.43963701;
    ASSERT_TRUE(matRM.isApprox(m, 0.0001)); //check if the transpose function of eurler angle works
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

TEST(test_euler, eulertoQuaternion) {
    reef_msgs::EulerAngle euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"121");
    Eigen::Matrix<double, 4, 1> mat = euler.toQuaternion();
    Eigen::Matrix<double, 4, 1> m;
    m << 0.4819681, 0.21883383, -0.07926117, 0.8447107;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"123");
    mat = euler.toQuaternion();
    m <<  0.17268843, 0.17564642, 0.4246233, 0.87121874 ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"131");
    mat = euler.toQuaternion();
    m <<   0.4819681 , 0.07926117, 0.21883383,0.8447107;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"132");
    mat = euler.toQuaternion();
    m <<   -0.02191637,  0.38854084,  0.24534186,0.88789811;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"212");
    mat = euler.toQuaternion();
    m <<   0.21883383, 0.4819681 , 0.07926117, 0.8447107;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"213");
    mat = euler.toQuaternion();
    m <<    0.24534186, -0.02191637,  0.38854084, 0.88789811;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"231");
    mat = euler.toQuaternion();
    m <<  0.4246233 , 0.17268843, 0.17564642, 0.87121874;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"232");
    mat = euler.toQuaternion();
    m <<   -0.07926117,  0.4819681 ,  0.21883383,0.8447107;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"312");
    mat = euler.toQuaternion();
    m <<  0.17564642, 0.4246233 , 0.17268843, 0.87121874;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"313");
    mat = euler.toQuaternion();
    m <<    0.21883383, -0.07926117,  0.4819681, 0.8447107 ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"321");
    mat = euler.toQuaternion();
    m <<    0.38854084,  0.24534186, -0.02191637, 0.88789811;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"323");
    mat = euler.toQuaternion();
    m <<   0.07926117, 0.21883383, 0.4819681, 0.8447107 ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
}


TEST(test_euler, eulertoAxisAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_euler.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("eulerAngle1212axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle1212axisAngle").at("eulerAngle121");
            std::string output = it.at("eulerAngle1212axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "121");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle1212axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle1212axisAngle").at("eulerAngle121");
            std::string output = it.at("eulerAngle1212axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "121");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle1232axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle1232axisAngle").at("eulerAngle123");
            std::string output = it.at("eulerAngle1232axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "123");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle1312axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle1312axisAngle").at("eulerAngle131");
            std::string output = it.at("eulerAngle1312axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "131");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle1322axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle1322axisAngle").at("eulerAngle132");
            std::string output = it.at("eulerAngle1322axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "132");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle2122axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle2122axisAngle").at("eulerAngle212");
            std::string output = it.at("eulerAngle2122axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "212");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle2132axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle2132axisAngle").at("eulerAngle213");
            std::string output = it.at("eulerAngle2132axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "213");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle2312axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle2312axisAngle").at("eulerAngle231");
            std::string output = it.at("eulerAngle2312axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "231");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle2322axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle2322axisAngle").at("eulerAngle232");
            std::string output = it.at("eulerAngle2322axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "232");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle3122axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle3122axisAngle").at("eulerAngle312");
            std::string output = it.at("eulerAngle3122axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "312");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle3132axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle3132axisAngle").at("eulerAngle313");
            std::string output = it.at("eulerAngle3132axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "313");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle3212axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle3212axisAngle").at("eulerAngle321");
            std::string output = it.at("eulerAngle3212axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "321");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("eulerAngle3232axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> eulerAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("eulerAngle3232axisAngle").at("eulerAngle323");
            std::string output = it.at("eulerAngle3232axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(eulerAngleInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::EulerAngle eulerAngle = reef_msgs::EulerAngle(eulerAngleInput, "323");
            Eigen::Matrix<double, 4, 1> mat = eulerAngle.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }

    }
}


TEST(test_euler, eulertoRodriguezParameter) {
    reef_msgs::EulerAngle euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"121");
    Eigen::Matrix<double, 3, 1> mat = euler.toRodriguezParameter();
    Eigen::Matrix<double, 3, 1> m;
    m << 0.5705718 ,  0.25906364, -0.09383232;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"123");
    mat = euler.toRodriguezParameter();
    m <<  0.19821478, 0.20161001, 0.48739    ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"131");
    mat = euler.toRodriguezParameter();
    m <<   0.5705718 , 0.09383232, 0.25906364;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"132");
    mat = euler.toRodriguezParameter();
    m <<   -0.02468342,  0.4375962 ,  0.27631759;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"212");
    mat = euler.toRodriguezParameter();
    m <<   0.25906364, 0.5705718 , 0.09383232;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"213");
    mat = euler.toRodriguezParameter();
    m <<     0.27631759, -0.02468342,  0.4375962 ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"231");
    mat = euler.toRodriguezParameter();
    m <<  0.48739   , 0.19821478, 0.20161001;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"232");
    mat = euler.toRodriguezParameter();
    m <<   -0.09383232,  0.5705718 ,  0.25906364;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"312");
    mat = euler.toRodriguezParameter();
    m <<  0.20161001, 0.48739   , 0.19821478;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"313");
    mat = euler.toRodriguezParameter();
    m <<     0.25906364, -0.09383232,  0.5705718  ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"321");
    mat = euler.toRodriguezParameter();
    m <<     0.4375962 ,  0.27631759, -0.02468342;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
    euler = reef_msgs::EulerAngle(0.1710, 0.4698, 0.8660,"323");
    mat = euler.toRodriguezParameter();
    m <<   0.09383232, 0.25906364, 0.5705718  ;
    ASSERT_TRUE(mat.isApprox(m, 0.0001));
}
