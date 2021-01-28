//
// Created by paul on 1/7/21.
//


#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <fstream>
#include "../include/reef_msgs/DCM.h"

#include "./include/test_utilities.h"


TEST(test_DCM, DCMtoaxisAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_DCM.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("DCM2axisAngle")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("DCM2axisAngle").at("DCM");
            std::string output = it.at("DCM2axisAngle").at("axisAngle");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 4, 1> mat = dcm.toAxisAngle();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_DCM, axisAngletoEulerAngle) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_DCM.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("DCM2euler121")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler121").at("DCM");
            std::string output = it.at("DCM2euler121").at("euler121");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("121");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler123")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler123").at("DCM");
            std::string output = it.at("DCM2euler123").at("euler123");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("123");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler131")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler131").at("DCM");
            std::string output = it.at("DCM2euler131").at("euler131");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("131");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler132")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler132").at("DCM");
            std::string output = it.at("DCM2euler132").at("euler132");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("132");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler212")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler212").at("DCM");
            std::string output = it.at("DCM2euler212").at("euler212");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("212");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler213")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler213").at("DCM");
            std::string output = it.at("DCM2euler213").at("euler213");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("213");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler231")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler231").at("DCM");
            std::string output = it.at("DCM2euler231").at("euler231");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("231");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler232")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler232").at("DCM");
            std::string output = it.at("DCM2euler232").at("euler232");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("232");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler312")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler312").at("DCM");
            std::string output = it.at("DCM2euler312").at("euler312");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("312");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler313")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler313").at("DCM");
            std::string output = it.at("DCM2euler313").at("euler313");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("313");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler321")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler321").at("DCM");
            std::string output = it.at("DCM2euler321").at("euler321");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("321");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
    for (auto& it : j) {
        if(it.contains("DCM2euler323")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2euler323").at("DCM");
            std::string output = it.at("DCM2euler323").at("euler323");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toEulerAngle("323");
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
}

TEST(test_DCM, axisAngletoQuaternion) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_DCM.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("DCM2quaternion")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,4,1> Output;
            std::string input =  it.at("DCM2quaternion").at("DCM");
            std::string output = it.at("DCM2quaternion").at("quaternion");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,4, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 4, 1> mat = dcm.toQuaternion();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }

}

TEST(test_DCM, axisRodriguezParameter) {
    std::ifstream i("../../../reef_msgs/test/data_test/test_DCM.json");
    nlohmann::json j;
    i >> j;
    for (auto& it : j) {
        if(it.contains("DCM2rodriguezParameter")) {
            std::cout<< it << std::endl;
            Eigen::Matrix<double,3,3> DCMInput;
            Eigen::Matrix<double,3,1> Output;
            std::string input =  it.at("DCM2rodriguezParameter").at("DCM");
            std::string output = it.at("DCM2rodriguezParameter").at("rodriguezParameter");
            std::cout << input<<std::endl;
            test_utilities::str_to_EigenMatrix(DCMInput, 3, 3, input,"," );
            test_utilities::str_to_EigenMatrix(Output,3, 1, output,"," );
            reef_msgs::DCM dcm = reef_msgs::DCM(DCMInput);
            Eigen::Matrix<double, 3, 1> mat = dcm.toRodriguezParameter();
            ASSERT_TRUE(mat.isApprox(Output, 0.0001));
        }
    }
}
