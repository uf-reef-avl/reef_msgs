//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_EULERANGLE_H
#define REEF_MSGS_EULERANGLE_H
#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    class EulerAngle : public reef_msgs::AngleRepresentationInterface {
    public:
        EulerAngle(const double &_yaw, const double &_pitch, const double &_roll);

        EulerAngle(const Eigen::Matrix<double, 3, 1> &_eulerAngle);

        EulerAngle(const EulerAngle &other);

        void setEulerAngle(const double &_yaw, const double &_pitch, const double &_roll);

        auto getEulerAngle() const -> const Eigen::Matrix<double, 3, 1> &;

        auto yaw() const -> double;

        void setYaw(const double &_yaw);

        auto pitch() const -> double;

        void setPitch(const double &_pitch);

        auto roll() const -> double;

        void setRoll(const double &_roll);

        EulerAngle &operator=(const EulerAngle &_other);

        friend std::ostream &operator<<(std::ostream &os, const EulerAngle &_inst);

        ~EulerAngle() {};

    private:
        Eigen::Matrix<double, 3, 1> m_EA; //!< this matrix store the euler angler value in the "yaw, pitch, roll"/ X,Y Z order
    };

}

#endif //REEF_MSGS_EULERANGLE_H
