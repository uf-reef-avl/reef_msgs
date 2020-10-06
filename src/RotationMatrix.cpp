//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/RotationMatrix.h"

namespace reef_msgs {

    auto RotationMatrix::getRotationMatrix() -> Eigen::Matrix<double, 3, 3> & {
        return m_RotationMatrix;
    }

    RotationMatrix::RotationMatrix(const Eigen::Matrix<double, 3, 3> &_RotationMatrix)
            : reef_msgs::AngleRepresentationInterface() {
        m_RotationMatrix = _RotationMatrix;
    }

    RotationMatrix::RotationMatrix(RotationMatrix &_other) {
        m_RotationMatrix = _other.getRotationMatrix();
    }


    RotationMatrix &RotationMatrix::operator=(RotationMatrix &_other) {
        m_RotationMatrix = _other.getRotationMatrix();
    }

    std::ostream &operator<<(std::ostream &os, const RotationMatrix &_inst) {
        os << "RotationMatrix { " << _inst.m_RotationMatrix(0, 0) << " , " << _inst.m_RotationMatrix(0, 1) << " , "
           << _inst.m_RotationMatrix(0, 2) << " , " << std::endl
           << _inst.m_RotationMatrix(1, 0) << " , " << _inst.m_RotationMatrix(1, 1) << " , "
           << _inst.m_RotationMatrix(1, 2) << " , " << std::endl
           << _inst.m_RotationMatrix(2, 0) << " , " << _inst.m_RotationMatrix(2, 1) << " , "
           << _inst.m_RotationMatrix(2, 2) << " , " << std::endl << " } ";
        return os;
    }
}

#include "reef_msgs/RotationMatrix.h"
