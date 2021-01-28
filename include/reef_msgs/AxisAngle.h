//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_AXISANGLE_H
#define REEF_MSGS_AXISANGLE_H
#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    /*!
    @class The AxisAngle is a c++ angle representation which stores a vector (x,y,z,angle) corresponding to a axis angle as data.
     It provides different methods to permit to convert itself to other angle representation as DCM, Quaternion, EulerAngle, RotationMatrix and RodriguezParameter
    */
    class AxisAngle : public reef_msgs::AngleRepresentationInterface {
    public:
        /*!
        @brief Constructor of a Axis Angle instance with x, y, z, angle as input argument
        @param[in] _x  the x axis of the unit vector
        @param[in] _y  the y axis of the unit vector
        @param[in] _z  the z axis of the unit vector
        @param[in] _angle  the rotation angle around the axis
        @return newly created Axis Angle instance
        */
        //add checking magnitude == 1 and generate z if not present
        AxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle);
        /*!
        @brief Constructor of a Axis Angle instance with a (x, y, z) vector and a angle as input argument
        @param[in] _axis  the unit vector (x,y,z) of the axis angle
        @param[in] _angle  the rotation angle around the axis
        @return newly created Axis Angle instance
        */
        AxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle);
        /*!
        @brief Constructor of a Axis Angle instance with a (x, y, z, angle)  as input argument
        @param[in] _axisAngle  a 4D vector which defines a axisangle with elements in this order (x, y, z, angle)
        @return newly created Axis Angle instance
        */
        AxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle);
        /*!
        @brief Copy constructor of a AxisAngle based on another AxisAngle
        @param[in] _other  the axis angle instance to copy
        @return newly created Axis Angle instance based on the copy of the axis angle input
        */
        AxisAngle(const AxisAngle &_other);
        /*!
        @brief Setter of the AxisAngle data with  4 double: x,y,z,angle as input
        @param[in] _x  the x axis of the unit vector
        @param[in] _y  the y axis of the unit vector
        @param[in] _z  the z axis of the unit vector
        @param[in] _angle  the rotation angle around the axis
        */
        void setAxisAngle(const double &_x, const double &_y, const double &_z, const double &_angle);
        /*!
        @brief Setter of the AxisAngle data with  1 vector (x,y,x) and one double angle as input
        @param[in] _axis  the unit vector (x,y,z) of the axis angle
        @param[in] _angle  the rotation angle around the axis
        */
        void setAxisAngle(const Eigen::Matrix<double, 3, 1> &_axis, const double &_angle);
        /*!
        @brief Setter of the AxisAngle data with  4 d vector (x,y,z,angle) as input
        @param[in] _axisAngle  a 4D vector which defines a axisangle with elements in this order (x, y, z, angle)
        */
        void setAxisAngle(const Eigen::Matrix<double, 4, 1> &_axisAngle);
        /*!
        @brief Getter of the AxisAngle data which corresponds to a 4D eigen vector (x, y, z, angle)
        @return a Eigen 4D vector which defines a axisangle with elements in this order (x, y, z, angle)
        */
        auto getAxisAngle() const -> const Eigen::Matrix<double, 4, 1> &;
        /*!
        @brief Getter of the x element of the AxisAngle unit vector data
        @return a double corresponding of the x element of the unit vector of the axis angle
        */
        auto x() const -> double;
        /*!
        @brief Setter of the x element of the AxisAngle unit vector data
        @@param[in] _x a double corresponding of the x element of the unit vector of the axis angle
        */
        void setX(const double &_x);
        /*!
        @brief Getter of the y element of the AxisAngle unit vector data
        @return a double corresponding of the y element of the unit vector of the axis angle
        */
        auto y() const -> double;
        /*!
        @brief Setter of the y element of the AxisAngle unit vector data
        @@param[in] _y a double corresponding of the y element of the unit vector of the axis angle
        */
        void setY(const double &_y);
        /*!
        @brief Getter of the z element of the AxisAngle unit vector data
        @return a double corresponding of the z element of the unit vector of the axis angle
        */
        auto z() const -> double;
        /*!
        @brief Setter of the z element of the AxisAngle unit vector data
        @@param[in] _z a double corresponding of the z element of the unit vector of the axis angle
        */
        void setZ(const double &_Z);
        /*!
        @brief Getter of the rotation angle of the axis angle
        @return a double corresponding of the rotation angle of the axis angle
        */
        auto angle() const -> double;
        /*!
        @brief Setter of the rotation angle of the AxisAngle unit vector data
        @@param[in] _angle a double corresponding of the rotation angle of the axis angle
        */
        void setAngle(const double &_angle);
        /*!
        @brief Normalize the vector unit of the axis angle in order to keep a magnitude of 1
        */
        void normalize();
        /*!
        @brief Translates the principal rotation vector into corresponding principal rotation element
        @return a 4D vector which corresponds to the principal rotation element
        */
        auto toPrincipalRotationElement() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the axis angle into a euler angle representation. The order of the rotation is set as an input
        @param[in] _eulerTransformation a string corresponding to the order of rotation around the axis
        @return a 3D vector which corresponds to the appropriate euler angle (x,y,z) depending on the order of rotation.
        */
        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Translates the axis angle into a rotation matrix
        @return a 3*3 rotation matrix which corresponds to the axis angle
        */
        auto toRotationMatrix() -> Eigen::Matrix3d;
        /*!
        @brief Translates the axis angle into a quaternion
        @return a 4d vector quaternion (x,y,z,w) which corresponds to the axis angle
        */
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the axis angle into a Rodriguez parameter
        @return a 3d vector Rodriguez Parameter (x,y,z) which corresponds to the axis angle
        */
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Translates the axis angle into a direction cosine matrix
        @return a 3*3 direction cosine matrix which corresponds to the axis angle
        */
        auto toDCM() -> Eigen::Matrix3d;
        /*!
        @brief Overload of the = operator to copy data from another instance
        @param[in] _other  the other AxisAngle instance and its data to copy
        @return newly created AxisAngle based on the data of the input
        */
        AxisAngle &operator=(const AxisAngle &_other);
        /*!
        @brief Overload of the << operator in order to print the information of a AxisAngle instance as a string
        @param[in] _os  the stream which retrieve the axis angle information as a string
        @param[in] _inst  the AxisAngle instance to print
        @return the axis angle information as a stream
        */
        friend std::ostream &operator<<(std::ostream &_os, const AxisAngle &_inst);
        /*!
        @brief Destructor of the axisangle instance
        */
        ~AxisAngle() {};
    private:
        Eigen::Matrix<double, 4, 1> m_axisAngle; //!< this matrix store the axis vector value in the 3 first container "xyz" order, the last container is for the angle

    };
}

#endif //REEF_MSGS_AXISANGLE_H
