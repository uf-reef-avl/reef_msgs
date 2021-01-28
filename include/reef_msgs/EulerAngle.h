//
// Created by paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_EULERANGLE_H
#define REEF_MSGS_EULERANGLE_H

#include "AngleRepresentationInterface.h"


namespace reef_msgs {
    /*!
@class The Eulerangle is a c++ angle representation which stores a vector (yaw,pitch,roll) and string eulerTransformation corresponding to a euler angle and one of the 12 rotation order (example:"321", "121" ... ) as data.
 It provides different methods to permit to convert itself to other angle representation as DCM, Quaternion, Axisangle, RotationMatrix and RodriguezParameter
*/
    class EulerAngle : public reef_msgs::AngleRepresentationInterface {
    public:
        /*!
        @brief Constructor of a Euler Angle instance with yaw, pitch, roll and the rotation order string as input argument
        @param[in] _yaw  the yaw axis of the unit vector
        @param[in] _picth  the pitch axis of the unit vector
        @param[in] _roll  the roll axis of the unit vector
        @param[in] _eulerTransformation  the rotation order applied
        @return newly created Euler Angle instance
        */
        EulerAngle(const double &_yaw, const double &_pitch, const double &_roll, const std::string &_eulerTransformation);
        /*!
        @brief Constructor of  Euler Angle instance with yaw, pitch, roll vector and the rotation order string as input argument
        @param[in] _eulerAngle  the unit vector (yaw,pitch,roll) vector corresponding to the euler angle
        @param[in] _eulerTransformation  the rotation order applied
        @return newly created Euler Angle instance
        */
        EulerAngle(const Eigen::Matrix<double, 3, 1> &_eulerAngle, const std::string &_eulerTransformation);
        /*!
        @brief Copy constructor of a EulerAngler based on another EulerAngle
        @param[in] _other  the euler angle instance to copy
        @return newly created Euler Angle instance based on the copy of the euler angle input
        */
        EulerAngle(const EulerAngle &other);
        /*!
        @brief Setter of the Euler angle data with  3 double: yaw, pitch, roll as input
        @param[in] _yaw  the yaw axis of the unit vector
        @param[in] _pitch  the pitch axis of the unit vector
        @param[in] _roll  the roll axis of the unit vector
        */
        void setEulerAngle(const double &_yaw, const double &_pitch, const double &_roll);
        /*!
        @brief Getter of the EulerAngle data which corresponds to a 3D eigen vector (yaw,pitch, roll)
        @return a Eigen 4D vector which defines a axisangle with elements in this order (x, y, z, angle)
        */
        auto getEulerAngle() const -> const Eigen::Matrix<double, 3, 1> &;
        /*!
        @brief Setter of the rotation order of the euler angle with the transformation string as input
        @param[in] _eulerTransformation  the eulerTransformation string which corresponds to the rotation order (example: "321", "121" ...)
        */
        void setEulerTransformation(const std::string &_eulerTransformation);
        /*!
        @brief Getter of the rotation order of euler angle as a string
        @return a string which corresponds to the rotation order of the euler angle
        */
        auto getEulerTransformation() const -> const std::string &;
        // TODO: finish documentation on euler angle
        auto yaw() const -> double;
        void setYaw(const double &_yaw);
        auto pitch() const -> double;
        void setPitch(const double &_pitch);
        auto roll() const -> double;
        void setRoll(const double &_roll);

        auto toDCM() -> Eigen::Matrix3d;
        auto toRotationMatrix() -> Eigen::Matrix3d;
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;

        EulerAngle &operator=(const EulerAngle &_other);
        friend std::ostream &operator<<(std::ostream &os, const EulerAngle &_inst);

        ~EulerAngle() {};

    private:
        Eigen::Matrix<double, 3, 1> m_EA; //!< this matrix store the euler angler value in the "yaw, pitch, roll"/
        std::string m_eulerTransformation;
    };

}

#endif //REEF_MSGS_EULERANGLE_H
