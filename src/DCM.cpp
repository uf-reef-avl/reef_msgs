//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/DCM.h"
#include "../include/reef_msgs/Quaternion.h"


namespace reef_msgs {

    auto DCM::getDCM() -> Eigen::Matrix<double, 3, 3> & {
        return m_DCM;
    }

    DCM::DCM(const Eigen::Matrix<double, 3, 3> &_DCM) : reef_msgs::AngleRepresentationInterface() {
        m_DCM = _DCM;
    }

    DCM::DCM(const DCM &_other) {
        m_DCM = _other.m_DCM;
    }

    auto DCM::toEulerAngle(const std::string &_eulerTransformation) -> Eigen::Matrix<double, 3, 1> {
        Eigen::Matrix<double, 3, 1> EA;
        if (_eulerTransformation == "121") {
            EA(0, 0) = atan2(m_DCM(0, 1), -m_DCM(0, 2));
            EA(1, 0) = acos(m_DCM(0, 0));
            EA(2, 0) = atan2(m_DCM(1, 0), m_DCM(2, 0));
        } else if (_eulerTransformation == "123") {
            EA(0, 0) = atan2(-m_DCM(2, 1), m_DCM(2, 2));
            EA(1, 0) = asin(m_DCM(2, 0));
            EA(2, 0) = atan2(-m_DCM(1, 0), m_DCM(0, 0));
        } else if (_eulerTransformation == "131") {
            EA(0, 0) = atan2(m_DCM(0, 2), m_DCM(0, 1));
            EA(1, 0) = acos(m_DCM(0, 0));
            EA(2, 0) = atan2(m_DCM(2, 0), -m_DCM(1, 0));
        } else if (_eulerTransformation == "132") {
            EA(0, 0) = atan2(m_DCM(1, 2), m_DCM(1, 1));
            EA(1, 0) = asin(-m_DCM(1, 0));
            EA(2, 0) = atan2(m_DCM(2, 0), m_DCM(0, 0));
        } else if (_eulerTransformation == "212") {
            EA(0, 0) = atan2(m_DCM(1, 0), m_DCM(1, 2));
            EA(1, 0) = acos(m_DCM(1, 1));
            EA(2, 0) = atan2(m_DCM(0, 1), -m_DCM(2, 1));
        } else if (_eulerTransformation == "213") {
            EA(0, 0) = atan2(m_DCM(2, 0), m_DCM(2, 2));
            EA(1, 0) = asin(-m_DCM(2, 1));
            EA(2, 0) = atan2(m_DCM(0, 1), m_DCM(1, 1));
        } else if (_eulerTransformation == "231") {
            EA(0, 0) = atan2(-m_DCM(0, 2), m_DCM(0, 0));
            EA(1, 0) = asin(m_DCM(0, 1));
            EA(2, 0) = atan2(-m_DCM(2, 1), m_DCM(1, 1));
        } else if (_eulerTransformation == "232") {
            EA(0, 0) = atan2(m_DCM(1, 2), -m_DCM(1, 0));
            EA(1, 0) = acos(m_DCM(1, 1));
            EA(2, 0) = atan2(m_DCM(2, 1), m_DCM(0, 1));
        } else if (_eulerTransformation == "312") {
            EA(0, 0) = atan2(-m_DCM(1, 0), m_DCM(1, 1));
            EA(1, 0) = asin(m_DCM(1, 2));
            EA(2, 0) = atan2(-m_DCM(0, 2), m_DCM(2, 2));
        } else if (_eulerTransformation == "313") {
            EA(0, 0) = atan2(m_DCM(2, 0), -m_DCM(2, 1));
            EA(1, 0) = acos(m_DCM(2, 2));
            EA(2, 0) = atan2(m_DCM(0, 2), m_DCM(1, 2));
        } else if (_eulerTransformation == "321") {
            EA(0, 0) = atan2(m_DCM(0, 1), m_DCM(0, 0));
            EA(1, 0) = asin(-m_DCM(0, 2));
            EA(2, 0) = atan2(m_DCM(1, 2), m_DCM(2, 2));
        } else if (_eulerTransformation == "323") {
            EA(0, 0) = atan2(m_DCM(2, 1), m_DCM(2, 0));
            EA(1, 0) = acos(m_DCM(2, 2));
            EA(2, 0) = atan2(m_DCM(1, 2), -m_DCM(0, 2));
        }
        return EA;
    }

    auto DCM::toRotationMatrix() -> Eigen::Matrix3d {
        return m_DCM.transpose();
    }

    auto DCM::toAxisAngle() -> Eigen::Matrix<double, 4, 1> {
        auto cp = (m_DCM.trace()-1)/2;
        auto p = acos(cp);
        auto sp = p/2/sin(p);
        Eigen::Matrix<double,4,1> q;

        auto x = (m_DCM(2,0) - m_DCM(0,2))*sp;
        auto y = (m_DCM(0,1) - m_DCM(1,0))*sp;
        auto z = sqrt( 1  - x*x - y*y);
        auto phi = (m_DCM(1,2) - m_DCM(2,1))*sp;
        q << x,y,z,phi;
        return q;
    }

//    //to quaternion in fact
//    auto DCM::toQuaternion() -> Eigen::Matrix<double, 4, 1> {
//        Eigen::Matrix<double, 4, 1> q;
//        double tr = m_DCM.trace();
//        Eigen::Matrix<double, 4, 1> b2;
//        b2(0, 0) = (1 + tr) / 4;
//        b2(1, 0) = (1 + 2 * m_DCM(0, 0) - tr) / 4;
//        b2(2, 0) = (1 + 2 * m_DCM(1, 1) - tr) / 4;
//        b2(3, 0) = (1 + 2 * m_DCM(2, 2) - tr) / 4;
//        int maxIndex = 0;
//        double max = b2[0];
//        for (int i = 0; i < 4; i++) {
//            if (b2(i, 0) > max) {
//                max = b2(i, 0);
//                maxIndex = i;
//            }
//        }
//        Eigen::Matrix<double, 4, 1> b;
//        b << b2;
//        if (maxIndex == 0) {
//            b(0, 0) = sqrt(b2(0, 0));
//            b(1, 0) = (m_DCM(1, 2) - m_DCM(2, 1)) / 4 / b(0, 0);
//            b(2, 0) = (m_DCM(2, 0) - m_DCM(0, 2)) / 4 / b(0, 0);
//            b(3, 0) = (m_DCM(0, 1) - m_DCM(1, 0)) / 4 / b(0, 0);
//        } else if (maxIndex == 1) {
//            b(1, 0) = sqrt(b2(1, 0));
//            b(0, 0) = (m_DCM(1, 2) - m_DCM(2, 1)) / 4 / b(1, 0);
//            if (b(0, 0) < 0) {
//                b(1, 0) = -b(1, 0);
//                b(0, 0) = -b(0, 0);
//                b(2, 0) = (m_DCM(0, 1) + m_DCM(1, 0)) / 4 / b(1, 0);
//                b(3, 0) = (m_DCM(2, 0) + m_DCM(0, 2)) / 4 / b(1, 0);
//            }
//        } else if (maxIndex == 2) {
//            b(2, 0) = sqrt(b2(2, 0));
//            b(0, 0) = (m_DCM(2, 0) - m_DCM(0, 2)) / 4 / b(2, 0);
//            if (b(0, 0) < 0) {
//                b(2, 0) = -b(2, 0);
//                b(0, 0) = -b(0, 0);
//                b(1, 0) = (m_DCM(0, 1) + m_DCM(1, 0)) / 4 / b(2, 0);
//                b(3, 0) = (m_DCM(1, 2) + m_DCM(2, 1)) / 4 / b(2, 0);
//            }
//        } else if (maxIndex == 3) {
//            b(3, 0) = sqrt(b2[3, 0]);
//            b(0, 0) = (m_DCM(0, 1) - m_DCM(1, 0)) / 4 / b(3, 0);
//            if (b(0, 0) < 0) {
//                b(3, 0) = -b(3, 0);
//                b(0, 0) = -b(0, 0);
//                b(1, 0) = (m_DCM(2, 0) + m_DCM(0, 2)) / 4 / b(3, 0);
//                b(2, 0) = (m_DCM(1, 2) + m_DCM(2, 1)) / 4 / b(3, 0);
//            }
//        }
//        return b;
//    }

    auto DCM::toQuaternion() -> Eigen::Matrix<double, 4, 1> {
        auto gamma = m_DCM.trace();
        double w2 = (1 + gamma) / 4;
        Eigen::Matrix<double, 3, 1> Ckk = m_DCM.diagonal();
        Eigen::Matrix<double, 3, 1> quat2temp =
                (Eigen::Matrix<double, 3, 1>::Ones() + 2 * Ckk - (gamma * Eigen::Matrix<double, 3, 1>::Ones())) / 4.;
        Eigen::Matrix<double, 4, 1> quat2;
        quat2 << quat2temp(0, 0), quat2temp(1, 0), quat2temp(2, 0), w2;
        int maxIndex = 0;
        double max = quat2[0];
        for (int i = 0; i < 4; i++) {
            if (quat2(i, 0) > max) {
                max = quat2(i, 0);
                maxIndex = i;
            }
        }
        Eigen::Matrix<double, 4, 1> q;
        q.setZero();
        q(maxIndex, 0) = sqrt(quat2(maxIndex, 0));
        double d = 4. * q[maxIndex];
        if (maxIndex == 3) {
            q(0, 0) = (m_DCM(1, 2) - m_DCM(2, 1)) / d;
            q(1, 0) = (m_DCM(2, 0) - m_DCM(0, 2)) / d;
            q(2, 0) = (m_DCM(0, 1) - m_DCM(1, 0)) / d;
        } else if (maxIndex == 0) {
            q(3, 0) = (m_DCM(1, 2) - m_DCM(2, 1)) / d;
            q(1, 0) = (m_DCM(0, 1) + m_DCM(1, 0)) / d;
            q(2, 0) = (m_DCM(2, 0) + m_DCM(0, 2)) / d;
        } else if (maxIndex == 1) {
            q(3, 0) = (m_DCM(2, 0) - m_DCM(0, 2)) / d;
            q(0, 0) = (m_DCM(0, 1) + m_DCM(1, 0)) / d;
            q(2, 0) = (m_DCM(1, 2) + m_DCM(2, 1)) / d;
        } else if (maxIndex == 2) {
            q(3, 0) = (m_DCM(0, 1) - m_DCM(1, 0)) / d;
            q(0, 0) = (m_DCM(2, 0) + m_DCM(0, 2)) / d;
            q(1, 0) = (m_DCM(1, 2) + m_DCM(2, 1)) / d;
        }
        q.normalize();
        return q;
    }

    auto DCM::toRodriguezParameter() -> Eigen::Matrix<double, 3, 1> {
        return Quaternion(toQuaternion()).toRodriguezParameter();
    }


    DCM &DCM::operator=(const DCM &_other) {
        m_DCM = _other.m_DCM;
    }

    std::ostream &operator<<(std::ostream &os, const DCM &_inst) {
        os << "DCM { " << _inst.m_DCM(0, 0) << " , " << _inst.m_DCM(0, 1) << " , " << _inst.m_DCM(0, 2) << " , "
           << std::endl
           << _inst.m_DCM(1, 0) << " , " << _inst.m_DCM(1, 1) << " , " << _inst.m_DCM(1, 2) << " , " << std::endl
           << _inst.m_DCM(2, 0) << " , " << _inst.m_DCM(2, 1) << " , " << _inst.m_DCM(2, 2) << " , " << std::endl
           << " } ";
        return os;
    }
}

#include "reef_msgs/DCM.h"
