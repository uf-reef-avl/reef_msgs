//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/DCM.h"

namespace reef_msgs {
 
    auto DCM::getDCM() -> Eigen::Matrix<double, 3, 3> & {
        return m_DCM;
    }

    DCM::DCM(const Eigen::Matrix<double, 3, 3> &_DCM) : reef_msgs::AngleRepresentationInterface() {
        m_DCM = _DCM;
    }

    DCM::DCM( DCM &_other) {
        m_DCM = _other.getDCM();
    }


    DCM & DCM::operator=( DCM & _other) {
        m_DCM = _other.getDCM();
    }

    std::ostream & operator<<(std::ostream &os, const DCM &_inst) {
        os << "DCM { " << _inst.m_DCM(0,0) << " , " << _inst.m_DCM(0,1) <<" , " << _inst.m_DCM(0,2) << " , " << std::endl
                << _inst.m_DCM(1,0) << " , " << _inst.m_DCM(1,1) <<" , " << _inst.m_DCM(1,2) << " , " << std::endl
                << _inst.m_DCM(2,0) << " , " << _inst.m_DCM(2,1) <<" , " << _inst.m_DCM(2,2) << " , " << std::endl << " } ";
        return os;
    }
}
#include "reef_msgs/DCM.h"
