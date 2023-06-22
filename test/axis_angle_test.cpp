//
// Created by paul on 10/29/20.
//

#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <fstream>
#include "../include/reef_msgs/AxisAngle.h"



#include "./include/test_utilities.h"


TEST(test_axisAngle, axisAngletoDCM) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_axisangle.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("axisAngle2DCM")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,3> Output;
            std::string input =  it.at("axisAngle2DCM").at("axisAngle");
            std::string output = it.at("axisAngle2DCM").at("DCM");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 3, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 3> mat = axisAngle.toDCM();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_axisAngle, axisAngletoEulerAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_axisangle.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("axisAngle2eulerAngle121")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle121").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle121").at("eulerAngle121");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("121");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle123")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle123").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle123").at("eulerAngle123");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("123");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle131")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle131").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle131").at("eulerAngle131");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("131");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle132")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle132").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle132").at("eulerAngle132");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("132");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle212")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle212").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle212").at("eulerAngle212");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("212");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle213")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle213").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle213").at("eulerAngle213");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("213");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle231")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle231").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle231").at("eulerAngle231");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("231");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle232")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle232").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle232").at("eulerAngle232");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("232");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle312")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle312").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle312").at("eulerAngle312");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("312");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle313")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle313").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle313").at("eulerAngle313");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("313");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle321")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle321").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle321").at("eulerAngle321");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("321");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("axisAngle2eulerAngle323")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2eulerAngle323").at("axisAngle");
            std::string output = it.at("axisAngle2eulerAngle323").at("eulerAngle323");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toEulerAngle("323");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_axisAngle, axisAngletoQuaternion) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_axisangle.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("axisAngle2quaternion")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("axisAngle2quaternion").at("axisAngle");
            std::string output = it.at("axisAngle2quaternion").at("quaternion");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 4, 1> mat = axisAngle.toQuaternion();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_axisAngle, axisRodriguezParameter) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_axisangle.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("axisAngle2rodriguezParameter")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,4,1> axisAngleInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("axisAngle2rodriguezParameter").at("axisAngle");
            std::string output = it.at("axisAngle2rodriguezParameter").at("rodriguezParameter");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(axisAngleInput, 4, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::AxisAngle axisAngle = reef_msgs::AxisAngle(axisAngleInput);
            Eigen::Matrix<double, 3, 1> mat = axisAngle.toRodriguezParameter();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
}