//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_DCM_H
#define REEF_MSGS_DCM_H

#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    /*!
    @class The DCM is a c++ angle representation which stores a 3*3 matrix corresponding to direction cosine matrix as data.
     It provides different methods to permit to convert itself to other angle representation as AxisAngle, Quaternion, EulerAngle, RotationMatrix and RodriguezParameter
    */
    class DCM : public reef_msgs::AngleRepresentationInterface {
    public:
        /*!
        @brief Constructor of a DCM instance with a 3*3 matrix  as input argument
        @param[in] _DCM  a 3*3 matrix which defines a direction consine matrix
        @return newly created DCM instance
        */
        DCM(const Eigen::Matrix<double, 3, 3> &_DCM);
        /*!
        @brief Copy constructor of a DCM based on another DCM
        @param[in] _other  the DCM instance to copy
        @return newly created DCM instance based on the copy of the DCM input
        */
        DCM(const DCM &_other);
        /*!
        @brief Getter of the Direction cosine matrix data which corresponds to a 3*3 eigen matrix
        @return a 3*3 eigen matrix which defines a Direction Cosine Matrix
        */
        auto getDCM() -> Eigen::Matrix<double, 3, 3> &;
        /*!
        @brief Overload of the = operator to copy data from another instance
        @param[in] _other  the other DCM instance and its data to copy
        @return newly created DCM based on the data of the input
        */
        DCM &operator=( const DCM &_other);
        /*!
        @brief Translates the Direction cosine matrix into a euler angle representation. The order of the rotation is set as an input
        @param[in] _eulerTransformation a string corresponding to the order of rotation around the axis
        @return a 3D vector which corresponds to the appropriate euler angle (x,y,z) depending on the order of rotation.
        */
        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Translates the DCM into a rotation matrix
        @return a 3*3 rotation matrix which corresponds to the DCM
        */
        auto toRotationMatrix() -> Eigen::Matrix3d;
        /*!
        @brief Translates the Direction Cosine Matrix into a axis angle
        @return a 4d vector axis angle (x,y,z,angle) which corresponds to the Direction Cosine Matrix
        */
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the Direction Cosine Matrix into a quaternion
        @return a 4d vector quaternion (x,y,z,w) which corresponds to the Direction Cosine Matrix
        */
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the Direction cosine matrix into a Rodriguez parameter
        @return a 3d vector Rodriguez Parameter (x,y,z) which corresponds to the Direction Cosine Matrix
        */
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Overload of the << operator in order to print the information of a DCM instance as a string
        @param[in] _os  the stream which retrieve the DCM information as a string
        @param[in] _inst  the DCM instance to print
        @return the DCM information as a stream
        */
        friend std::ostream &operator<<(std::ostream &os, const DCM &_inst);
        /*!
        @brief Destructor of the DCM instance
        */
        ~DCM() {};

        Eigen::Matrix<double, 3, 3> m_DCM; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_DCM_H
