//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/EulerAngle.h"
#include "../include/reef_msgs/Quaternion.h"


namespace reef_msgs {
    EulerAngle::EulerAngle(const EulerAngle &_other) {
        m_EA = _other.getEulerAngle();
        m_eulerTransformation = _other.getEulerTransformation();
    }

    EulerAngle::EulerAngle(const double &_yaw, const double &_pitch, const double &_roll, const std::string &_eulerTransformation):AngleRepresentationInterface(){
        setEulerAngle(_yaw,_pitch,_roll);
        setEulerTransformation(_eulerTransformation);
    }

    EulerAngle::EulerAngle(const Eigen::Matrix<double, 3, 1> &_eulerAngle, const std::string &_eulerTransformation):AngleRepresentationInterface(){
        setEulerAngle(_eulerAngle(0,0),_eulerAngle(1,0),_eulerAngle(2,0));
        setEulerTransformation(_eulerTransformation);
    }

    void EulerAngle::setEulerAngle(const double &_yaw, const double &_pitch, const double &_roll) {
            m_EA(0, 0) = _yaw;
            m_EA(1, 0) = _pitch;
            m_EA(2, 0) = _roll;
/*
        if(_eulerTransformation == "121") {
        }else if(_eulerTransformation == "123"){
        }else if(_eulerTransformation == "131"){
        }else if(_eulerTransformation == "132"){
        }else if(_eulerTransformation == "212"){
        }else if(_eulerTransformation == "213"){
        }else if(_eulerTransformation == "231"){
        }else if(_eulerTransformation == "232"){
        }else if(_eulerTransformation == "312"){
        }else if(_eulerTransformation == "313"){
        }else if(_eulerTransformation == "321") {
        }else if(_eulerTransformation == "323"){
        } else{
            throw std::invalid_argument( "this euler transformation isn't known" );
        }*/
    }
    auto EulerAngle::getEulerAngle() const -> const Eigen::Matrix<double, 3, 1> & {
        return m_EA;
    }
    void EulerAngle::setEulerTransformation(const std::string &_eulerTransformation) {
        m_eulerTransformation = _eulerTransformation;

    }
    auto EulerAngle::getEulerTransformation() const -> const std::string & {
        return m_eulerTransformation;
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

    auto EulerAngle::toDCM() -> Eigen::Matrix3d {
        auto st1 = sin(m_EA(0,0));
        auto ct1 = cos(m_EA(0,0));
        auto st2 = sin(m_EA(1,0));
        auto ct2 = cos(m_EA(1,0));
        auto st3 = sin(m_EA(2,0));
        auto ct3 = cos(m_EA(2,0));
        Eigen::Matrix<double,3, 3> C;

        if(m_eulerTransformation == "121") {
            C(0,0) = ct2;
            C(0,1) = st1*st2;
            C(0,2) = -ct1*st2;
            C(1,0) = st2*st3;
            C(1,1) = ct1*ct3-ct2*st1*st3;
            C(1,2) = ct3*st1+ct1*ct2*st3;
            C(2,0) = ct3*st2;
            C(2,1) = -ct2*ct3*st1-ct1*st3;
            C(2,2) = ct1*ct2*ct3-st1*st3;
        }else if(m_eulerTransformation == "123"){
            C(0,0) = ct2*ct3;
            C(0,1) = ct3*st1*st2+ct1*st3;
            C(0,2) = st1*st3-ct1*ct3*st2;
            C(1,0) = -ct2*st3;
            C(1,1) = ct1*ct3-st1*st2*st3;
            C(1,2) = ct3*st1+ct1*st2*st3;
            C(2,0) = st2;
            C(2,1) = -ct2*st1;
            C(2,2) = ct1*ct2;
        }else if(m_eulerTransformation == "131"){
            C(0,0) = ct2;
            C(0,1) = ct1*st2;
            C(0,2) = st1*st2;
            C(1,0) = -ct3*st2;
            C(1,1) = ct1*ct2*ct3-st1*st3;
            C(1,2) = ct2*ct3*st1+ct1*st3;
            C(2,0) = st2*st3;
            C(2,1) = -ct3*st1-ct1*ct2*st3;
            C(2,2) = ct1*ct3-ct2*st1*st3;
        }else if(m_eulerTransformation == "132"){
            C(0,0) = ct2*ct3;
            C(0,1) = ct1*ct3*st2+st1*st3;
            C(0,2) = ct3*st1*st2-ct1*st3;
            C(1,0) = -st2;
            C(1,1) = ct1*ct2;
            C(1,2) = ct2*st1;
            C(2,0) = ct2*st3;
            C(2,1) = -ct3*st1+ct1*st2*st3;
            C(2,2) = ct1*ct3+st1*st2*st3;
        }else if(m_eulerTransformation == "212") {
            C(0,0) = ct1*ct3-ct2*st1*st3;
            C(0,1) = st2*st3;
            C(0,2) = -ct3*st1-ct1*ct2*st3;
            C(1,0) = st1*st2;
            C(1,1) = ct2;
            C(1,2) = ct1*st2;
            C(2,0) = ct2*ct3*st1+ct1*st3;
            C(2,1) = -ct3*st2;
            C(2,2) = ct1*ct2*ct3-st1*st3;
        }else if(m_eulerTransformation == "213"){
            C(0,0) = ct1*ct3+st1*st2*st3;
            C(0,1) = ct2*st3;
            C(0,2) = -ct3*st1+ct1*st2*st3;
            C(1,0) = ct3*st1*st2-ct1*st3;
            C(1,1) = ct2*ct3;
            C(1,2) = ct1*ct3*st2 + st1*st3;
            C(2,0) = ct2*st1;
            C(2,1) = -st2;
            C(2,2) = ct1*ct2;
        }else if(m_eulerTransformation == "231"){
            C(0,0) = ct1*ct2;
            C(0,1) = st2;
            C(0,2) = -ct2*st1;
            C(1,0) = -ct1*ct3*st2+st1*st3;
            C(1,1) = ct2*ct3;
            C(1,2) = ct3*st1*st2+ct1*st3;
            C(2,0) = ct3*st1+ct1*st2*st3;
            C(2,1) = -ct2*st3;
            C(2,2) = ct1*ct3-st1*st2*st3;
        }else if(m_eulerTransformation == "232"){
            C(0,0) = ct1*ct2*ct3-st1*st3;
            C(0,1) = ct3*st2;
            C(0,2) = -ct2*ct3*st1-ct1*st3;
            C(1,0) = -ct1*st2;
            C(1,1) = ct2;
            C(1,2) = st1*st2;
            C(2,0) = ct3*st1+ct1*ct2*st3;
            C(2,1) = st2*st3;
            C(2,2) = ct1*ct3-ct2*st1*st3;
        }else if(m_eulerTransformation == "312"){
            C(0,0) = ct1*ct3-st1*st2*st3;
            C(0,1) = ct3*st1+ct1*st2*st3;
            C(0,2) = -ct2*st3;
            C(1,0) = -ct2*st1;
            C(1,1) = ct1*ct2;
            C(1,2) = st2;
            C(2,0) = ct3*st1*st2+ct1*st3;
            C(2,1) = st1*st3-ct1*ct3*st2;
            C(2,2) = ct2*ct3;
        }else if(m_eulerTransformation == "313"){
            C(0,0) = ct3*ct1-st3*ct2*st1;
            C(0,1) = ct3*st1+st3*ct2*ct1;
            C(0,2) = st3*st2;
            C(1,0) = -st3*ct1-ct3*ct2*st1;
            C(1,1) = -st3*st1+ct3*ct2*ct1;
            C(1,2) = ct3*st2;
            C(2,0) = st2*st1;
            C(2,1) = -st2*ct1;
            C(2,2) = ct2;
        }else if(m_eulerTransformation == "321") {
            C(0,0) = ct2*ct1;
            C(0,1) = ct2*st1;
            C(0,2) = -st2;
            C(1,0) = st3*st2*ct1-ct3*st1;
            C(1,1) = st3*st2*st1+ct3*ct1;
            C(1,2) = st3*ct2;
            C(2,0) = ct3*st2*ct1+st3*st1;
            C(2,1) = ct3*st2*st1-st3*ct1;
            C(2,2) = ct3*ct2;
        }else if(m_eulerTransformation == "323"){
            C(0,0) = ct1*ct2*ct3-st1*st3;
            C(0,1) = ct2*ct3*st1+ct1*st3;
            C(0,2) = -ct3*st2;
            C(1,0) = -ct3*st1-ct1*ct2*st3;
            C(1,1) = ct1*ct3-ct2*st1*st3;
            C(1,2) = st2*st3;
            C(2,0) = ct1*st2;
            C(2,1) = st1*st2;
            C(2,2) = ct2;
        }
        return C;
    }
    auto EulerAngle::toRotationMatrix() -> Eigen::Matrix3d {
        return toDCM().transpose();
    }
    auto EulerAngle::toAxisAngle() -> Eigen::Matrix<double,4,1> {
        return Quaternion(toQuaternion()).toAxisAngle();
    }
    auto EulerAngle::toQuaternion() -> Eigen::Matrix<double,4,1> {
        Eigen::Matrix<double,4,1> q;
        double q0, q1, q2, q3;
        if(m_eulerTransformation == "121") {
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  cos(e2)* sin(e1+e3);
            q2 =  sin(e2)* cos(e1-e3);
            q3 =  sin(e2)* sin(e1-e3);
        }else if(m_eulerTransformation == "123"){
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3-s1*s2*s3;
            q1 = s1*c2*c3+c1*s2*s3;
            q2 = c1*s2*c3-s1*c2*s3;
            q3 = c1*c2*s3+s1*s2*c3;
        }else if(m_eulerTransformation == "131"){
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  cos(e2)* sin(e1+e3);
            q2 =  sin(e2)* sin(-e1+e3);
            q3 =  sin(e2)* cos(-e1+e3);
        }else if(m_eulerTransformation == "132"){
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3+s1*s2*s3;
            q1 = s1*c2*c3-c1*s2*s3;
            q2 = c1*c2*s3-s1*s2*c3;
            q3 = c1*s2*c3+s1*c2*s3;
        }else if(m_eulerTransformation == "212"){
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  sin(e2)* cos(-e1+e3);
            q2 =  cos(e2)* sin(e1+e3);
            q3 =  sin(e2)* sin(-e1+e3);
        }else if(m_eulerTransformation == "213"){
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3+s1*s2*s3;
            q1 = c1*s2*c3+s1*c2*s3;
            q2 = s1*c2*c3-c1*s2*s3;
            q3 = c1*c2*s3-s1*s2*c3;
        }else if(m_eulerTransformation == "231"){
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3-s1*s2*s3;
            q1 = c1*c2*s3+s1*s2*c3;
            q2 = s1*c2*c3+c1*s2*s3;
            q3 = c1*s2*c3-s1*c2*s3;
        }else if(m_eulerTransformation == "232"){
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  sin(e2)* sin(e1-e3);
            q2 =  cos(e2)* sin(e1+e3);
            q3 =  sin(e2)* cos(e1-e3);
        }else if(m_eulerTransformation == "312"){
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3-s1*s2*s3;
            q1 = c1*s2*c3-s1*c2*s3;
            q2 = c1*c2*s3+s1*s2*c3;
            q3 = s1*c2*c3+c1*s2*s3;
        }else if(m_eulerTransformation == "313"){
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  sin(e2)* cos(e1-e3);
            q2 =  sin(e2)* sin(e1-e3);
            q3 =  cos(e2)* sin(e1+e3);
        }else if(m_eulerTransformation == "321") {
            auto c1 =  cos(m_EA(0,0)/2);
            auto s1 =  sin(m_EA(0,0)/2);
            auto c2 =  cos(m_EA(1,0)/2);
            auto s2 =  sin(m_EA(1,0)/2);
            auto c3 =  cos(m_EA(2,0)/2);
            auto s3 =  sin(m_EA(2,0)/2);

            q0 = c1*c2*c3+s1*s2*s3;
            q1 = c1*c2*s3-s1*s2*c3;
            q2 = c1*s2*c3+s1*c2*s3;
            q3 = s1*c2*c3-c1*s2*s3;
        }else if(m_eulerTransformation == "323"){
            auto e1 = m_EA(0,0)/2;
            auto e2 = m_EA(1,0)/2;
            auto e3 = m_EA(2,0)/2;

            q0 =  cos(e2)* cos(e1+e3);
            q1 =  sin(e2)* sin(-e1+e3);
            q2 =  sin(e2)* cos(-e1+e3);
            q3 =  cos(e2)* sin(e1+e3);
        }
        q << q1,q2,q3,q0;
        return q;
    }
    auto EulerAngle::toRodriguezParameter() -> Eigen::Matrix<double,3,1>{
        return Quaternion(toQuaternion()).toRodriguezParameter();
    }


    EulerAngle & EulerAngle::operator=(const EulerAngle & _other) {
        m_EA = _other.getEulerAngle();
        m_eulerTransformation = _other.getEulerTransformation();
    }

    std::ostream & operator<<(std::ostream &os, const EulerAngle &_inst) {
        os << "Euler angle: "<< _inst.m_eulerTransformation <<" :{ yaw : " << _inst.yaw() << ", pitch : " << _inst.pitch() <<", roll : " << _inst.roll() << " };" ;
        return os;
    }
}
#include "reef_msgs/EulerAngle.h"
