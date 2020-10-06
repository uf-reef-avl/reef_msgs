//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/AxisAngle.h"

namespace reef_msgs {

    void AxisAngle::setAxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle) {
        m_axisAngle(0, 0) = _axis(0, 0);
        m_axisAngle(1, 0) = _axis(1, 0);
        m_axisAngle(2, 0) = _axis(2, 0);
        m_axisAngle(3, 0) = _angle;
    }

    void AxisAngle::setAxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle) {
        m_axisAngle(0, 0) = _x;
        m_axisAngle(1, 0) = _y;
        m_axisAngle(2, 0) = _z;
        m_axisAngle(3, 0) = _angle;
    }

    void AxisAngle::setAxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle) {
        m_axisAngle = _axisAngle;
    }

    auto AxisAngle::getAxisAngle() const -> const Eigen::Matrix<double, 4, 1> & {
        return m_axisAngle;
    }

    auto AxisAngle::x() const -> double {
        return m_axisAngle(0, 0);
    }

    void AxisAngle::setX(const double &_x) {
        m_axisAngle(0, 0) = _x;
    }

    auto AxisAngle::y() const -> double {
        return m_axisAngle(1, 0);
    }

    void AxisAngle::setY(const double &_y) {
        m_axisAngle(1, 0) = _y;
    }

    auto AxisAngle::z() const -> double {
        return m_axisAngle(2, 0);
    }

    void AxisAngle::setZ(const double &_z) {
        m_axisAngle(2, 0) = _z;
    }

    auto AxisAngle::angle() const -> double {
        return m_axisAngle(3, 0);
    }

    void AxisAngle::setAngle(const double &_angle) {
        m_axisAngle(3, 0) = _angle;
    }

    AxisAngle::AxisAngle(const AxisAngle &_other) {
        m_axisAngle = _other.getAxisAngle();
    }

    AxisAngle::AxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle)
            : AngleRepresentationInterface() {
        setAxisAngle(_x, _y, _z, _angle);
    }

    AxisAngle::AxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle)
            : AngleRepresentationInterface(), m_axisAngle(_axisAngle) {
    }

    AxisAngle::AxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle) : AngleRepresentationInterface() {
        setAxisAngle(_axis, _angle);
    }

    AxisAngle &AxisAngle::operator=(const AxisAngle &_other) {
        m_axisAngle = _other.getAxisAngle();
    }

    std::ostream &operator<<(std::ostream &os, const AxisAngle &_inst) {
        os << "Axis Angle { x : " << _inst.x() << ", y : " << _inst.y() << ", z : " << _inst.z() << ", angle : "
           << _inst.angle() << " };";
        return os;
    }
}
