//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_AXISANGLE_H
#define REEF_MSGS_AXISANGLE_H
#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    class AxisAngle : public reef_msgs::AngleRepresentationInterface {
    public:

    public:
        //add checking magnitude == 1 and generate z if not present
        AxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle);
        AxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle);
        AxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle);
        AxisAngle(const AxisAngle &other);

        void setAxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle);
        void setAxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle);
        void setAxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle);
        auto getAxisAngle() const -> const Eigen::Matrix<double, 4, 1> &;
        auto x() const -> double;
        void setX(const double &_x);
        auto y() const -> double;
        void setY(const double &_y);
        auto z() const -> double;
        void setZ(const double &_Z);
        auto angle() const -> double;
        void setAngle(const double &_angle);

        void normalize();
        auto toPrincipalRotationElement() -> Eigen::Matrix<double,4,1>;
        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        auto toRotationMatrix() -> Eigen::Matrix3d;
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;
        auto toDCM() -> Eigen::Matrix3d;
        AxisAngle &operator=(const AxisAngle &_other);

        friend std::ostream &operator<<(std::ostream &os, const AxisAngle &_inst);

        ~AxisAngle() {};
    private:
        Eigen::Matrix<double, 4, 1> m_axisAngle; //!< this matrix store the axis vector value in the 3 first container "xyz" order, the last container is for the angle

    };
}

#endif //REEF_MSGS_AXISANGLE_H
