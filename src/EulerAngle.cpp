//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/EulerAngle.h"


namespace reef_msgs {
    EulerAngle::EulerAngle(const EulerAngle &_other) {
        m_EA = _other.getEulerAngle();
    }

    EulerAngle::EulerAngle(const double &_yaw, const double &_pitch, const double &_roll):AngleRepresentationInterface(){
        setEulerAngle(_yaw,_pitch,_roll);
    }

    void EulerAngle::setEulerAngle(const double &_yaw, const double &_pitch, const double &_roll) {
        m_EA(0,0) = _yaw;
        m_EA(1,0) = _pitch;
        m_EA(2,0) = _roll;
    }
    auto EulerAngle::getEulerAngle() const -> const Eigen::Matrix<double, 3, 1> & {
        return m_EA;
    }

    auto EulerAngle::yaw() const -> double {
        return m_EA(0,0);
    }
    void EulerAngle::setYaw(const double &_yaw) {
        m_EA(0,0) = _yaw;
    }
    auto EulerAngle::pitch() const -> double {
        return m_EA(1,0);
    }
    void EulerAngle::setPitch(const double &_pitch) {
        m_EA(1,0) = _pitch;
    }
    auto EulerAngle::roll() const ->  double {
        return m_EA(2,0);
    }
    void EulerAngle::setRoll(const double &_roll) {
        m_EA(2, 0) = _roll;
    }

    EulerAngle & EulerAngle::operator=(const EulerAngle & _other) {
        m_EA = _other.getEulerAngle();
    }

    std::ostream & operator<<(std::ostream &os, const EulerAngle &_inst) {
        os << "Euler angle { yaw : " << _inst.yaw() << ", pitch : " << _inst.pitch() <<", roll : " << _inst.roll() << " };" ;
        return os;
    }
}
#include "reef_msgs/EulerAngle.h"
