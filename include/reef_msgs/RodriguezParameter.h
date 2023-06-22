//
// Created bb paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_RODRIGUEZPARAMETER_H
#define REEF_MSGS_RODRIGUEZPARAMETER_H
#include "AngleRepresentationInterface.h"

namespace reef_msgs{
    /*!
    @class RodriguezParameter

    @brief The RodriguezParameter is a c++ angle representation which stores a vector (x,y,z,angle) corresponding to a axis angle as data.
    It provides different methods to permit to convert itself to other angle representation as DCM, Quaternion, EulerAngle, RotationMatrix and AxisAngle
    */
    class RodriguezParameter: public AngleRepresentationInterface{
    public:
        /*!
        @brief Constructor of a Rodriguez Parameter instance with x, y, z as input argument
        @param[in] _x  the x axis of the unit vector
        @param[in] _y  the y axis of the unit vector
        @param[in] _z  the z axis of the unit vector
        @return newly created Rodriguez Parameter instance
        */
        RodriguezParameter(const double &_x, const double &_y, const double &_z);
        /*!
        @brief Constructor of a Rodriguez Parameter instance with a eigen matrix as input argument
        @param[in] _parameters  the (x,y,z) parameters in a eigen vector
        @return newly created Rodriguez Parameter instance
        */
        RodriguezParameter(const Eigen::Matrix<double, 3, 1> &_parameters);
        /*!
        @brief Copy constructor of a RodriguezParameter based on another RodriguezParameter
        @param[in] _other  the RodriguezParameter instance to copy
        @return newly created RodriguezParameter instance based on the copy of the Rodriguez Parameter input
        */
        RodriguezParameter(const RodriguezParameter &other);
        /*!
        @brief Setter of the Rodrgiuez paramette data with x, y, z as input argument
        @param[in] _x  the x axis of the unit vector
        @param[in] _y  the y axis of the unit vector
        @param[in] _z  the z axis of the unit vector
        */
        void setParameter(const double &_x, const double &_y, const double &_z);
        /*!
        @brief Getter of the elements of the Rodriguez Parameter unit vector data
        @return a eigen 3D vector corresponding of the (x,y,z) composing a rodriguez parameter
        */
        auto getParameter() const -> const Eigen::Matrix<double,3,1>&;
        /*!
        @brief Getter of the x element of the Rodriguez Parameter unit vector data
        @return a double corresponding of the x element of the unit vector of the Rodriguez Parameter
        */
        auto x() const ->  double ;
        /*!
        @brief Setter of the x element of the Rodriguez Parameter unit vector data
        @@param[in] _x a double corresponding of the x element of the unit vector of the Rodriguez Parameter
        */
        void setX(const double &_x);
        /*!
        @brief Getter of the y element of the Rodriguez Parameter unit vector data
        @return a double corresponding of the y element of the unit vector of the Rodriguez Parameter
        */
        auto y() const -> double;
        /*!
        @brief Setter of the y element of the Rodriguez Parameter unit vector data
        @@param[in] _y a double corresponding of the x element of the unit vector of the Rodriguez Parameter
        */
        void setY(const double &_y);
        /*!
        @brief Getter of the z element of the Rodriguez Parameter unit vector data
        @return a double corresponding of the z element of the unit vector of the Rodriguez Parameter
        */
        auto z() const-> double;
        /*!
        @brief Setter of the z element of the Rodriguez Parameter unit vector data
        @@param[in] _z a double corresponding of the x element of the unit vector of the Rodriguez Parameter
        */
        void setZ(const double &_z);
        /*!
        @brief Translates the Rodriguez parameter into a euler angle representation. The order of the rotation is set as an input
        @param[in] _eulerTransformation a string corresponding to the order of rotation around the axis
        @return a 3D vector which corresponds to the appropriate euler angle (x,y,z) depending on the order of rotation.
        */
        auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Translates the Rodriguez parameter into a rotation matrix
        @return a 3*3 rotation matrix which corresponds to the Rodriguez Parameter
        */
        auto toRotationMatrix() -> Eigen::Matrix3d;
        /*!
        @brief Translates the Rodriguez parameter into a axis angle
        @return a 4d vector axis angle (x,y,z,angle) which corresponds to the Rodriguez Parameter
        */
        auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the Rodriguez parameter into a quaternion
        @return a 4d vector quaternion (x,y,z,w) which corresponds to the Rodriguez Parameter
        */
        auto toQuaternion() -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Translates the ROdriguez Parameter into a direction cosine matrix
        @return a 3*3 direction cosine matrix which corresponds to the rodriguez parameter
        */
        auto toDCM() -> Eigen::Matrix<double,3,3>;
        /*!
        @brief Overload of the = operator to copy data from another instance
        @param[in] _other  the other RodriguezParameter instance and its data to copy
        @return newly created RodirguezParameter based on the data of the input
        */
        RodriguezParameter& operator=(const RodriguezParameter & _other);
        /*!
        @brief Overload of the << operator in order to print the information of a RodriguezParameter instance as a string
        @param[in] _os  the stream which retrieve the rodriguez parameter information as a string
        @param[in] _inst  the RodriguezParameter instance to print
        @return the rodriguez parameter information as a stream
        */
        friend std::ostream& operator<<(std::ostream& os,const RodriguezParameter & _inst);
        /*!
        @brief Destructor of the rodriguez parameter instance
        */
        ~RodriguezParameter() {};
    private:
        Eigen::Matrix<double,3,1> m_parameter; //!< this matrix store the parameters value in the "xyz" order

    };

};



#endif //REEF_MSGS_RODRIGUEZPARAMETER_H
