//
// Created by paul on 10/5/20.
//

#include "reef_msgs/RodriguezParameter.h"
#include "reef_msgs/Quaternion.h"

namespace reef_msgs {

    void RodriguezParameter::setParameter(const double &_x, const double &_y, const double &_z) {
        m_parameter(0, 0) = _x;
        m_parameter(1, 0) = _y;
        m_parameter(2, 0) = _z;
    }

    auto RodriguezParameter::getParameter() const -> const Eigen::Matrix<double, 3, 1> & {
        return m_parameter;
    }

    auto RodriguezParameter::x() const -> double {
        return m_parameter(0, 0);
    }

    void RodriguezParameter::setX(const double &_x) {
        m_parameter(0, 0) = _x;
    }

    auto RodriguezParameter::y() const -> double {
        return m_parameter(1, 0);
    }

    void RodriguezParameter::setY(const double &_y) {
        m_parameter(1, 0) = _y;
    }

    auto RodriguezParameter::z() const -> double {
        return m_parameter(2, 0);
    }

    void RodriguezParameter::setZ(const double &_z) {
        m_parameter(2, 0) = _z;
    }

    RodriguezParameter::RodriguezParameter(const RodriguezParameter &_other) {
        m_parameter = _other.getParameter();
    }

    RodriguezParameter::RodriguezParameter(const double &_x, const double &_y, const double &_z)
            : AngleRepresentationInterface() {
        setParameter(_x, _y, _z);
    }

    RodriguezParameter::RodriguezParameter(const Eigen::Matrix<double, 3, 1> &_quat)
            : AngleRepresentationInterface(), m_parameter(_quat) {}

    RodriguezParameter &RodriguezParameter::operator=(const RodriguezParameter &_other) {
        m_parameter = _other.getParameter();
    }

    auto RodriguezParameter::toEulerAngle(const std::string &_eulerTransformation) -> Eigen::Matrix<double, 3, 1> {
        return reef_msgs::Quaternion(toQuaternion()).toEulerAngle(_eulerTransformation);
    }

    auto RodriguezParameter::toRotationMatrix() -> Eigen::Matrix3d {
        return toDCM().transpose();
    }

    auto RodriguezParameter::toAxisAngle() -> Eigen::Matrix<double, 4, 1> {
        return reef_msgs::Quaternion(toQuaternion()).toAxisAngle();
    }

    auto RodriguezParameter::toQuaternion() -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 1> q;
        q(3, 0) = 1 / sqrt(1 + (m_parameter.transpose() * m_parameter)(0, 0));
        q(0, 0) = m_parameter(0, 0) * q(3, 0);
        q(1, 0) = m_parameter(1, 0) * q(3, 0);
        q(2, 0) = m_parameter(2, 0) * q(3, 0);

        return q;

    }

    auto RodriguezParameter::toDCM() -> Eigen::Matrix<double, 3, 3> {

        auto q1 = m_parameter(0, 0);
        auto q2 = m_parameter(1, 0);
        auto q3 = m_parameter(2, 0);
        auto d1 = (m_parameter.transpose() * m_parameter)(0, 0);
        Eigen::Matrix<double, 3, 3> C;
        C(0, 0) = 1 + 2 * q1 * q1 - d1;
        C(0, 1) = 2 * (q1 * q2 + q3);
        C(0, 2) = 2 * (q1 * q3 - q2);
        C(1, 0) = 2 * (q2 * q1 - q3);
        C(1, 1) = 1 + 2 * q2 * q2 - d1;
        C(1, 2) = 2 * (q2 * q3 + q1);
        C(2, 0) = 2 * (q3 * q1 + q2);
        C(2, 1) = 2 * (q3 * q2 - q1);
        C(2, 2) = 1 + 2 * q3 * q3 - d1;
        C = C / (1 + d1);
        return C;

    }

    std::ostream &operator<<(std::ostream &os, const RodriguezParameter &_inst) {
        os << "Rodriguez parameters { x : " << _inst.x() << ", y : " << _inst.y() << ", z : " << _inst.z() << " };";
        return os;
    }
}
