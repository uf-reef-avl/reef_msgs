//
// Created by paul on 10/5/20.
//

#include "reef_msgs/RodriguezParameter.h"

namespace reef_msgs{

    void RodriguezParameter::setParameter(const double &_x, const double &_y, const double &_z) {
        m_parameter(0,0) = _x;
        m_parameter(1,0) = _y;
        m_parameter(2,0) = _z;
    }
    auto RodriguezParameter::getParameter() const -> const Eigen::Matrix<double, 3, 1> & {
        return m_parameter;
    }
    auto RodriguezParameter::x() const -> double {
        return m_parameter(0,0);
    }
    void RodriguezParameter::setX(const double &_x) {
        m_parameter(0,0) = _x;
    }
    auto RodriguezParameter::y() const ->  double {
        return m_parameter(1,0);
    }
    void RodriguezParameter::setY(const double &_y) {
        m_parameter(1,0) = _y;
    }
    auto RodriguezParameter::z() const ->  double {
        return m_parameter(2,0);
    }
    void RodriguezParameter::setZ(const double &_z) {
        m_parameter(2,0) = _z;
    }
    RodriguezParameter::RodriguezParameter(const RodriguezParameter &_other) {
        m_parameter = _other.getParameter();
    }

    RodriguezParameter::RodriguezParameter(const double &_x, const double &_y, const double &_z):AngleRepresentationInterface(){
        setParameter(_x,_y,_z);
    }
    RodriguezParameter::RodriguezParameter(const Eigen::Matrix<double, 3, 1> &_quat):AngleRepresentationInterface(), m_parameter(_quat) {}
    RodriguezParameter & RodriguezParameter::operator=(const RodriguezParameter & _other) {
        m_parameter = _other.getParameter();
    }
    std::ostream & operator<<(std::ostream &os, const RodriguezParameter &_inst) {
        os << "Rodriguez parameters { x : " << _inst.x() << ", y : " << _inst.y() <<", z : " << _inst.z() <<" };" ;
        return os;
    }
}
