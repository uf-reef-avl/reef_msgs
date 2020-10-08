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

        friend std::ostream &operator<<(std::ostream &os, const DCM &_inst);

        ~DCM() {};

        Eigen::Matrix<double, 3, 3> m_DCM; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_DCM_H
