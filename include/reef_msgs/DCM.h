//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_DCM_H
#define REEF_MSGS_DCM_H

#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    class DCM : public reef_msgs::AngleRepresentationInterface {
    public:
        DCM(const Eigen::Matrix<double, 3, 3> &_DCM);
        DCM(const DCM &other);
        auto getDCM() -> Eigen::Matrix<double, 3, 3> &;
        DCM &operator=( const DCM &_other);

        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        auto toRotationMatrix() -> Eigen::Matrix3d;
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;

        friend std::ostream &operator<<(std::ostream &os, const DCM &_inst);

        ~DCM() {};

        Eigen::Matrix<double, 3, 3> m_DCM; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_DCM_H
