//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_ROTATIONMATRIX_H
#define REEF_MSGS_ROTATIONMATRIX_H
#include "AngleRepresentationInterface.h"
#include "Quaternion.h"

namespace reef_msgs {
    class RotationMatrix : public reef_msgs::AngleRepresentationInterface {
    public:
        RotationMatrix(const Eigen::Matrix<double, 3, 3> &_RotationMatrix);

        RotationMatrix(const RotationMatrix &other);

        //to avoid to redefine all the function from eigen, we let the user access the matrix directly to change it;
        auto getRotationMatrix() -> Eigen::Matrix<double, 3, 3> &;

        RotationMatrix &operator=(const RotationMatrix &_other);

        friend std::ostream &operator<<(std::ostream &os, const RotationMatrix &_inst);

        ~RotationMatrix() {};

        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        auto toDCM() -> Eigen::Matrix3d;
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;


        Eigen::Matrix<double, 3, 3> m_RotationMatrix; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_ROTATIONMATRIX_H
