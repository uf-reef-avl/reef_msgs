//
// Created by paul on 10/5/20.
//

#include "reef_msgs/RodriguezParameter.h"

namespace reef_msgs{

    void RodriguezParameter::setParameter(const double &_a, const double &_b, const double &_c, const double &_d) {
        m_parameter(0,0) = _a;
        m_parameter(1,0) = _b;
        m_parameter(2,0) = _c;
        m_parameter(3,0) = _d;
    }
    auto RodriguezParameter::getParameter() const -> const Eigen::Matrix<double, 4, 1> & {
        return m_parameter;
    }
    auto RodriguezParameter::a() const -> double {
        return m_parameter(0,0);
    }
    void RodriguezParameter::setA(const double &_a) {
        m_parameter(0,0) = _a;
    }
    auto RodriguezParameter::b() const ->  double {
        return m_parameter(1,0);
    }
    void RodriguezParameter::setB(const double &_b) {
        m_parameter(1,0) = _b;
    }
    auto RodriguezParameter::c() const ->  double {
        return m_parameter(2,0);
    }
    void RodriguezParameter::setC(const double &_c) {
        m_parameter(2,0) = _c;
    }
    auto RodriguezParameter::d() const -> double {
        return m_parameter(3,0);
    }
    void RodriguezParameter::setD(const double &_d) {
        m_parameter(3,0) = _d;
    }
    RodriguezParameter::RodriguezParameter(const RodriguezParameter &_other) {
        m_parameter = _other.getParameter();
    }

    RodriguezParameter::RodriguezParameter(const double &_a, const double &_b, const double &_c, const double &_d):AngleRepresentationInterface(){
        setParameter(_a,_b,_c,_d);
    }
    RodriguezParameter::RodriguezParameter(const Eigen::Matrix<double, 4, 1> &_quat):AngleRepresentationInterface(), m_parameter(_quat) {}
    RodriguezParameter & RodriguezParameter::operator=(const RodriguezParameter & _other) {
        m_parameter = _other.getParameter();
    }
    std::ostream & operator<<(std::ostream &os, const RodriguezParameter &_inst) {
        os << "Rodriguez parameters { a : " << _inst.a() << ", b : " << _inst.b() <<", c : " << _inst.c() <<", d : " << _inst.d()<<" };" ;
        return os;
    }
}
#include "reef_msgs/RodriguezParameter.h"
