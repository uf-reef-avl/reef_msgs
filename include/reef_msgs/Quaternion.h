//
// Created by paulbuzaud on 9/17/20.
//
#pragma once
#ifndef REEF_MSGS_Quaternion_H
#define REEF_MSGS_Quaternion_H
#include "AngleRepresentationInterface.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include "dynamics.h"
#include "matrix_operation.h"


namespace reef_msgs{
    /*!
    @class Quaternion

    @Brief The Quaternion is a c++ angle representation which stores a vector (x,y,z,w) corresponding to a axis angle as data.
     It provides different methods to permit to convert itself to other angle representation as DCM, AxisAngle, EulerAngle, RotationMatrix and RodriguezParameter
    */
    class Quaternion : public AngleRepresentationInterface{

    public:
      //change it to a factory and return pointer instead of copy because if it is not well used it can change only one thing in the class. Have to test it
    /*!
    @brief Create a new instance of a real quaternion
    @return a newly created quaternion (x=0,y=0,z=0,w=1)
    */
      static auto eye() -> Quaternion;
        /*!
      @brief Create a new instance of a random parametrized quaternion
      @return a newly created random parametrized  quaternion
      */
      static auto rand() -> Quaternion;
        /*!
        @brief Multiplication of two Eigen 4D vector quaternions
        @param[in] _p  Eigen 4D vector quaternion (x,y,z,w)
        @param[in] _q  Eigen 4D vector quaternion (x,y,z,w)
        @return a Eigen 4D vector which is the product of the two other quaternions as input
        */
      static auto quaternionMultiplication(Eigen::Matrix<double,4,1> _p,Eigen::Matrix<double,4,1> _q) -> Eigen::Matrix<double,4,1>;
        /*!
      @brief Multiplication of two Eigen quaternions
      @param[in] _p  Eigen quaternion (x,y,z,w)
      @param[in] _q  Eigen quaternion (x,y,z,w)
      @return a Eigen quaternion which is the product of the two other quaternions as input
      */
      static auto quaternionMultiplication(Eigen::Quaterniond _p , Eigen::Quaterniond _q) -> Eigen::Quaterniond;
        /*!
        @brief Multiplication of two Eigen quaternions
        @param[in] _p  Eigen quaternion (x,y,z,w)
        @param[in] _q  Eigen quaternion (x,y,z,w)
        @return a Eigen quaternion which is the product of the two other quaternions as input
        */
      static auto quatMult(Eigen::Quaterniond _q , Eigen::Quaterniond _p) -> Eigen::Quaterniond;
        /*!
      @brief Transform a eigen quaternion to a eigen 3*3 matrix rotation matrix
      @param[in] _q  Eigen quaternion (x,y,z,w)
      @return a 3*3 rotation matrix which corresponds to the quaternion representation as input
      */
      static auto  quaternionToRotation(Eigen::Quaterniond _q) -> Eigen::Matrix3d;
        /*!
        @brief Transform a geometry message quaternion into a eigen 3*3 matrix rotation matrix
        @param[in] _q  Eigen quaternion (x,y,z,w)
        @return a 3*3 rotation matrix which corresponds to the quaternion representation as input
        */
      static auto  quaternionToRotation(geometry_msgs::Quaternion q) -> Eigen::Matrix3d;
        /*!
      @brief Transform a eigen 3*3 direction cosine matrix into a eigen 4d vector quaternion
      @param[in] _q  Geometry_msgs quaternion (x,y,z,w)
      @return a eigen 4d vector quaternion corresponds to the Eigen 3*3 representation as input
      */
      static auto  fromDCMtoQuaternion(const Eigen::Matrix<double,3,3> &_p) -> Eigen::Matrix<double,4,1>;
        /*!
        @brief Transform a axis angle into a eigen 4d vector quaternion
        @param[in] _angle the angle of the axis angle as input
        @param[in] _axis  the axis of the axis angle as input
        @return a Eigen 4d quaternion which corresponds to the quaternion representation as input
        */
      static auto  fromAngleAxistoQuaternion(const double &_angle, const Eigen::Matrix<double,3,1> &_axis) -> Eigen::Matrix<double,4,1>;

      /*!
      @brief Compute the norm of the quaternion
      @return a double which represents the norm of the quaternion
      */
      double norm();
        /*!
      @brief Normalize the (x,y,z,w) vector of the quaternion
      */
      void normalize();
        /*!
        @brief Remove the real part of the quaternion
        */
      void imagine();
        /*!
      @brief Return the quaternion as a Eigen 4d quaternion with its real part removed
      @return a Eigen 4d quaternion which has its real part removed
      */
      auto imaginaryEigenQuat()->Eigen::Matrix<double,4,1>;
        /*!
        @brief Return the quaternion as a reef_msgs Quaternion with its real part removed
        @return a reef_msgs quaternion which has its real part removed
        */
      auto imaginaryQuaternion()->Quaternion;
        /*!
      @brief Inverse the quaternion
      */
      void inverse();
        /*!
        @brief Return the quaternion as a inversed Eigen 4d quaternion
        @return a inversed Eigen 4d quaternion
        */
      auto inversedEigenQuat()-> Eigen::Matrix<double, 4, 1>;
        /*!
      @brief Return the quaternion as a inversed reef_msgs quaternion
      @return a inversed reef_msgs quaternion
      */
      auto inversedQuaternion()-> Quaternion;
        /*!
      @brief Translates the quaternion into a rotation matrix
      @return a 3*3 rotation matrix which corresponds to the quaternion
      */
      auto toRotation() -> Eigen::Matrix3d;
        /*!
      @brief Translates the quaternion into a direction cosine matrix
      @return a 3*3 direction cosine matrix which corresponds to the quaternion
      */
      auto toDCM() -> Eigen::Matrix3d;
        /*!
      @brief Translates the quaternion into a rotation matrix
      @return a 3*3 rotation matrix which corresponds to the quaternion
      */
      auto toRotationMatrix() -> Eigen::Matrix3d;
        /*!
        @brief Translates the quaternion into a axis angle
        @return a 4d vector axis angle (x,y,z,angle) which corresponds to the quaternion
        */
      auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
        /*!
      @brief Translates the quaternion into a euler angle representation. The order of the rotation is set as an input
      @param[in] _eulerTransformation a string corresponding to the order of rotation around the axis
      @return a 3D vector which corresponds to the appropriate euler angle (x,y,z) depending on the order of rotation.
      */
      auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
        /*!
      @brief Translates the quaternion into a Rodriguez parameter
      @return a 3d vector Rodriguez Parameter (x,y,z) which corresponds to quaternion
      */
      auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;
        /*!
        @brief Constructor of a quaternion instance with x, y, z, w as input argument
        @param[in] _x  the x element of the quaternion
        @param[in] _y  the y element of the quaternion
        @param[in] _z  the z element of the quaternion
        @param[in] _w  the w element of the quaternion
        @return newly created quaternion instance
        */
      Quaternion(const double &_x, const double &_y, const double &_z, const double &_w);
        /*!
        @brief Constructor of a quaternion instance with a eigen 3*3 direction cosine matrix as input argument
      @param[in] _p  a direction cosine matrix
      @return newly created quaternion instance
      */
      Quaternion(const Eigen::Matrix<double,3,3> &_p);//from DCM
        /*!
          @brief Constructor of a quaternion instance with a eigen 4d vector as input argument
        @param[in] _quat  a eigen 4d vector corresponding to a quaternion (x,y,z,w)
        @return newly created quaternion instance
        */
      Quaternion(const Eigen::Matrix<double, 4, 1> &_quat);
        /*!
        @brief Copy constructor of a quaternion instance
      @param[in] _other  another quaternion instance
      @return newly created quaternion instance
      */
      Quaternion(const Quaternion &_other);
        /*!
        @brief Setter of the quaternion data with x, y,  z, w as input
        @param[in] _x  the x element of the quaternion
        @param[in] _y  the y element of the quaternion
        @param[in] _z  the z element of the quaternion
        @param[in] _w  the w element of the quaternion
        */
      void setQuaternion(const double &_x, const double &_y, const double &_z, const double &_w);
        /*!
        @brief Getter of the quaternion data which corresponds to a 4D eigen vector (x, y, z, w)
        @return a Eigen 4D vector which defines a quaternion with elements in this order (x, y, z, w)
        */
      auto getQuaternion() const -> const Eigen::Matrix<double,4,1>&;
        /*!
      @brief Getter of the x element of the quaternion unit vector data
      @return a double corresponding of the x element of the unit vector of the quaternion
      */
      auto x() const ->  double ;
      /*!
      @brief Setter of the x element of the quaternion unit vector data
      @@param[in] _x a double corresponding of the x element of the unit vector of the quaternion
      */
      void setX(const double &_x);
        /*!
    @brief Getter of the x element of the quaternion unit vector data
    @return a double corresponding of the x element of the unit vector of the quaternion
    */
      auto y() const -> double;
        /*!
  @brief Setter of the y element of the quaternion unit vector data
  @@param[in] _y a double corresponding of the x element of the unit vector of the quaternion
  */
      void setY(const double &_y);
        /*!
    @brief Getter of the y element of the quaternion unit vector data
    @return a double corresponding of the y element of the unit vector of the quaternion
    */
      auto z() const-> double;
        /*!
    @brief Setter of the z element of the quaternion unit vector data
    @@param[in] _z a double corresponding of the x element of the unit vector of the quaternion
    */
      void setZ(const double &_z);
        /*!
        @brief Getter of the z element of the quaternion unit vector data
        @return a double corresponding of the z element of the unit vector of the quaternion
        */
      auto w() const -> double;
        /*!
        @brief Setter of the w element of the quaternion unit vector data
        @@param[in] _w a double corresponding of the x element of the unit vector of the quaternion
        */
      void setW(const double &_w);
        /*!
        @brief  Compute the Matrix that relates angular velocity to quaternion derivative. See Trawny eq. 20.
        @return a eigein 4*3 matrix corresponding to the matrix that relates angular velocity to quaternion derivative
        */
      auto Xi() -> Eigen::Matrix<double, 4, 3>;
        /*!
      @brief  Compute the "Psi" matrix as defined by Trawny eq. 19.
      @return a eigein 4*3 matrix corresponding to the "Psi" matrix as defined by Trawny eq. 19.
      */
      auto Psi() -> Eigen::Matrix<double, 4, 3>;
        /*!
        @brief Overload of the = operator to copy data from another instance
        @param[in] _other  the other quaternion instance and its data to copy
        @return newly created quaternion based on the data of the input
        */
      Quaternion& operator= (const Quaternion & _other);
        /*!
      @brief Overload of the << operator in order to print the information of a quaternion instance as a string
      @param[in] _os  the stream which retrieve the quaternion information as a string
      @param[in] _inst  the quaternion instance to print
      @return the quaternion information as a stream
      */
      friend std::ostream& operator<<(std::ostream& os,const Quaternion & _inst);
        /*!
      @brief Destructor of the quaternion instance
      */
      ~Quaternion() {};
    private:
      Eigen::Matrix<double,4,1> m_q; //!< this matrix store the quaternion value in the "xyzw" order

    };
    /*!
    @brief Overload of the * operator between two quaternion to multiply them
      @param[in] _a  first quaternion element of the quaternion multiplication
      @param[in] _b  second quaternion element of the quaternion multiplication
      @return the quaternion multiplication as a quaternion
    */
    inline Quaternion operator*(Quaternion _a, Quaternion const& _b) {
        // note 'a' is passed by value and thus copied
        _a = Quaternion::quaternionMultiplication(_a.getQuaternion(), _b.getQuaternion());
        return _a;
    }
};


#endif //REEF_MSGS_Quaternion_H
