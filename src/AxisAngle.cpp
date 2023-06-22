//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/AxisAngle.h"
#include "../include/reef_msgs/Quaternion.h"

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

    AxisAngle::AxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle)
            : AngleRepresentationInterface() {
        setAxisAngle(_axis, _angle);
    }

    void AxisAngle::normalize()  {
        auto norm = sqrt(m_axisAngle(0, 0)*m_axisAngle(0, 0) + m_axisAngle(1, 0)*m_axisAngle(1, 0) + m_axisAngle(2, 0)*m_axisAngle(2, 0)) ;
        m_axisAngle(0, 0) = m_axisAngle(0, 0) / norm;
        m_axisAngle(1, 0) = m_axisAngle(1, 0) / norm;
        m_axisAngle(2, 0) = m_axisAngle(2, 0) / norm;
    }

    auto AxisAngle::toPrincipalRotationElement() -> Eigen::Matrix<double, 4, 1> {
        //we use another convention for this
        Eigen::Matrix<double, 4, 1> q;
        q(0,0) = sqrt(m_axisAngle(0, 0)*m_axisAngle(0, 0) + m_axisAngle(1, 0)*m_axisAngle(1, 0) + m_axisAngle(3, 0)*m_axisAngle(3, 0)) ;
        q(1,0) = m_axisAngle(3, 0) / q(0,0);
        q(2,0) = m_axisAngle(0, 0) / q(0,0);
        q(3,0) = m_axisAngle(1, 0) / q(0,0);
        return q;
    }

    auto AxisAngle::toEulerAngle(const std::string &_eulerTransformation) -> Eigen::Matrix<double, 3, 1> {
        Eigen::Matrix<double, 3, 1> eulerAngle;
        eulerAngle = reef_msgs::Quaternion(toQuaternion()).toEulerAngle(_eulerTransformation);
        return eulerAngle;
    }

    auto AxisAngle::toRotationMatrix() -> Eigen::Matrix3d {
        return toDCM().transpose();
    }

    auto AxisAngle::toQuaternion() -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 1> q;
        auto q1 = toPrincipalRotationElement();
        auto sp = sin(q1(0, 0) / 2);
        //rework this hackich way to recreate proper quaternion convention (x,y,z,w) instead of (w,x,y,z)
        q(3, 0) = cos(q1(0, 0) / 2);
        q(0, 0) = q1(1, 0) * sp;
        q(1, 0) = q1(2, 0) * sp;
        q(2, 0) = q1(3, 0) * sp;

        return q;
    }

    auto AxisAngle::toRodriguezParameter() -> Eigen::Matrix<double, 3, 1> {
        Eigen::Matrix<double, 4, 1> q1 = toPrincipalRotationElement();
        auto tp = tan(q1(0, 0) / 2);
        Eigen::Matrix<double, 3, 1> q;
        q(0, 0) = q1(1, 0) * tp;
        q(1, 0) = q1(2, 0) * tp;
        q(2, 0) = q1(3, 0) * tp;

        return q;
    }

    auto AxisAngle::toDCM() -> Eigen::Matrix3d {
        auto q0 = sqrt(m_axisAngle(0, 0)*m_axisAngle(0, 0) + m_axisAngle(1, 0)*m_axisAngle(1, 0) + m_axisAngle(3, 0)*m_axisAngle(3, 0)) ;
        auto q1 = m_axisAngle(3, 0) / q0;
        auto q2 = m_axisAngle(0, 0) / q0;
        auto q3 = m_axisAngle(1, 0) / q0;

        auto cp = cos(q0);
        auto sp = sin(q0);
        auto d1 = 1 - cp;
        Eigen::Matrix<double, 3, 3> C;
        C(0, 0) = q1 * q1 * d1 + cp;
        C(0, 1) = q1 * q2 * d1 + q3 * sp;
        C(0, 2) = q1 * q3 * d1 - q2 * sp;
        C(1, 0) = q2 * q1 * d1 - q3 * sp;
        C(1, 1) = q2 * q2 * d1 + cp;
        C(1, 2) = q2 * q3 * d1 + q1 * sp;
        C(2, 0) = q3 * q1 * d1 + q2 * sp;
        C(2, 1) = q3 * q2 * d1 - q1 * sp;
        C(2, 2) = q3 * q3 * d1 + cp;

        return C;
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
