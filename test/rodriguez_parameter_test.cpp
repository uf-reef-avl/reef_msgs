//
// Created by paul on 1/7/21.
//


#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <fstream>
#include "../include/reef_msgs/RodriguezParameter.h"



#include "./include/test_utilities.h"


TEST(test_rodriguezParameter, rodriguezParametertoDCM) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_rodriguez.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("rodriguezParameter2DCM")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,3> Output;
            std::string input =  it.at("rodriguezParameter2DCM").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2DCM").at("DCM");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 3, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 3> mat = rodriguezParameter.toDCM();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_rodriguezParameter, rodriguezParametertoEulerAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_rodriguez.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("rodriguezParameter2eulerAngle121")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle121").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle121").at("eulerAngle121");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("121");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle123")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle123").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle123").at("eulerAngle123");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("123");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle131")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle131").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle131").at("eulerAngle131");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("131");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle132")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle132").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle132").at("eulerAngle132");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("132");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle212")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle212").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle212").at("eulerAngle212");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("212");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle213")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle213").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle213").at("eulerAngle213");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("213");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle231")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle231").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle231").at("eulerAngle231");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("231");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle232")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle232").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle232").at("eulerAngle232");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("232");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle312")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle312").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle312").at("eulerAngle312");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("312");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle313")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle313").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle313").at("eulerAngle313");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("313");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle321")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle321").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle321").at("eulerAngle321");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("321");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
        if(it.contains("rodriguezParameter2eulerAngle323")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("rodriguezParameter2eulerAngle323").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2eulerAngle323").at("eulerAngle323");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 3, 1> mat = rodriguezParameter.toEulerAngle("323");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_rodriguezParameter, rodriguezParametertoQuaternion) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_rodriguez.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("rodriguezParameter2quaternion")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("rodriguezParameter2quaternion").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2quaternion").at("quaternion");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 4, 1> mat = rodriguezParameter.toQuaternion();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_rodriguezParameter, rodriguezParametertoAxisAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_rodriguez.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("rodriguezParameter2rodriguezParameter")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,1> rodriguezParameterInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("rodriguezParameter2axisAngle").at("rodriguezParameter");
            std::string output = it.at("rodriguezParameter2axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(rodriguezParameterInput, 3, 1, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::RodriguezParameter rodriguezParameter = reef_msgs::RodriguezParameter(rodriguezParameterInput);
            Eigen::Matrix<double, 4, 1> mat = rodriguezParameter.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
}