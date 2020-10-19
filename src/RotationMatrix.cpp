//
// Created by paul on 10/5/20.
//
#include "../include/reef_msgs/RotationMatrix.h"
#include "../include/reef_msgs/DCM.h"

namespace reef_msgs {

    auto RotationMatrix::getRotationMatrix()  -> Eigen::Matrix<double, 3, 3> & {
        return m_RotationMatrix;
    }

    RotationMatrix::RotationMatrix(const Eigen::Matrix<double, 3, 3> &_RotationMatrix)
            : reef_msgs::AngleRepresentationInterface() {
        m_RotationMatrix = _RotationMatrix;
    }

    RotationMatrix::RotationMatrix(const RotationMatrix &_other) {
        m_RotationMatrix = _other.m_RotationMatrix;
    }

    auto RotationMatrix::toEulerAngle(const std::string &_eulerTransformation) -> Eigen::Matrix<double, 3, 1> {
        return reef_msgs::DCM(toDCM()).toEulerAngle(_eulerTransformation);
    }
    auto RotationMatrix::toDCM() -> Eigen::Matrix3d{
        return m_RotationMatrix.transpose();
    }
    auto RotationMatrix::toAxisAngle() -> Eigen::Matrix<double,4,1>{
        return reef_msgs::DCM(toDCM()).toAxisAngle();
    }
    auto RotationMatrix::toQuaternion() -> Eigen::Matrix<double,4,1>{
        return reef_msgs::DCM(toDCM()).toQuaternion();
    }
    auto RotationMatrix::toRodriguezParameter() -> Eigen::Matrix<double,3,1>{
        return reef_msgs::DCM(toDCM()).toRodriguezParameter();
    }


    RotationMatrix &RotationMatrix::operator=(const RotationMatrix &_other) {
        m_RotationMatrix = _other.m_RotationMatrix;
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

