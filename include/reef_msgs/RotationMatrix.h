//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_ROTATIONMATRIX_H
#define REEF_MSGS_ROTATIONMATRIX_H
#include "AngleRepresentationInterface.h"
#include "Quaternion.h"

namespace reef_msgs {
    /*!
    @class The RotationMatrix is a c++ angle representation which stores a 3*3 matrix corresponding to a rotation matrix as data.
     It provides different methods to permit to convert itself to other angle representation as AxisAngle, Quaternion, EulerAngle, DCM and RodriguezParameter
    */
    class RotationMatrix : public reef_msgs::AngleRepresentationInterface {
    public:
        /*!
        @brief Constructor of a Rotation Matrix instance with a 3*3 matrix as input argument
        @param[in] _RotationMatrix  a 3*3 matrix which defines a rotation matrix
        @return newly created Rotation Matrix instance
        */
        RotationMatrix(const Eigen::Matrix<double, 3, 3> &_RotationMatrix);
        /*!
        @brief Copy constructor of a Rotation matrix based on another Rotation matrix
        @param[in] _other  the Rotation matrix instance to copy
        @return newly created Rotation matrix instance based on the copy of the Rotation matrix input
        */
        RotationMatrix(const RotationMatrix &_other);
        /*!
        @brief Getter of the Rotation Matrix data which corresponds to a 3*3 eigen matrix
        @return a 3*3 eigen matrix which defines a Rotation Matrix
        */
        //to avoid to redefine all the function from eigen, we let the user access the matrix directly to change it;
        auto getRotationMatrix() -> Eigen::Matrix<double, 3, 3> &;
        /*!
        @brief Overload of the = operator to copy data from another instance
        @param[in] _other  the other rotation matrix instance and its data to copy
        @return newly created rotation matrix based on the data of the input
        */
        RotationMatrix &operator=(const RotationMatrix &_other);
        /*!
        @brief Overload of the << operator in order to print the information of a Rotation matrix instance as a string
        @param[in] _os  the stream which retrieve the Rotation matrix information as a string
        @param[in] _inst  the rotation matrix instance to print
        @return the rotation matrix information as a stream
        */
        friend std::ostream &operator<<(std::ostream &os, const RotationMatrix &_inst);
        /*!
        @brief Destructor of the Rotation Matrix instance
        */
        ~RotationMatrix() {};
        /*!
        @brief Translates the Rotation Matrix into a euler angle representation. The order of the rotation is set as an input
        @param[in] _eulerTransformation a string corresponding to the order of rotation around the axis
        @return a 3D vector which corresponds to the appropriate euler angle (x,y,z) depending on the order of rotation.
        */
        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Translates the rotation matrix into a DCM
        @return a 3*3 DCM  which corresponds to the rotation matrix
        */
        auto toDCM() -> Eigen::Matrix3d;
        /*!
        @brief Translates the Rotation Matrix into a axis angle
        @return a 4d vector axis angle (x,y,z,angle) which corresponds to the Rotation Matrix
        */
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the Rotation Matrix into a quaternion
        @return a 4d vector quaternion (x,y,z,w) which corresponds to the Direction Cosine Matrix
        */
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the rotation matrix into a Rodriguez parameter
        @return a 3d vector Rodriguez Parameter (x,y,z) which corresponds to the rotation matrix
        */
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;


        Eigen::Matrix<double, 3, 3> m_RotationMatrix; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_ROTATIONMATRIX_H
