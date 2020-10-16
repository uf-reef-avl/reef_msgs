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
    class Quaternion : public AngleRepresentationInterface{

    public:
      //change it to a factory and return pointer instead of copy because if it is not well used it can change only one thing in the class. Have to test it
      static auto eye() -> Quaternion;
      static auto rand() -> Quaternion;
      static auto quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q) -> Eigen::Matrix<double,4,1>;
      static auto quaternionMultiplication(Eigen::Quaterniond p , Eigen::Quaterniond q) -> Eigen::Quaterniond;
      static auto quatMult(Eigen::Quaterniond q , Eigen::Quaterniond p) -> Eigen::Quaterniond;
      static auto  quaternionToRotation(Eigen::Quaterniond q) -> Eigen::Matrix3d;
      static auto  quaternionToRotation(geometry_msgs::Quaternion q) -> Eigen::Matrix3d;
      static auto  fromDCMtoQuaternion(const Eigen::Matrix<double,3,3> &_p) -> Eigen::Matrix<double,4,1>;
      static auto  fromAngleAxistoQuaternion(const double &_angle, const Eigen::Matrix<double,3,1> &_axis) -> Eigen::Matrix<double,4,1>;

      double norm();
      void normalize();
      void imagine();
      auto imaginaryEigenQuat()->Eigen::Matrix<double,4,1>;
      auto imaginaryQuaternion()->Quaternion;
      void inverse();
      auto inversedEigenQuat()-> Eigen::Matrix<double, 4, 1>;
      auto inversedQuaternion()-> Quaternion;
      auto toRotation() -> Eigen::Matrix3d;
      auto toDCM() -> Eigen::Matrix3d;
      auto toRotationMatrix() -> Eigen::Matrix3d;
      auto toAxisAngle() -> Eigen::Matrix<double,4,1>;
      auto toEulerAngle(const std::string &_eulerTransformation ="321") -> Eigen::Matrix<double,3,1>;
      auto toRodriguezParameter() -> Eigen::Matrix<double,3,1>;

      Quaternion(const double &_x, const double &_y, const double &_z, const double &_w);
      Quaternion(const Eigen::Matrix<double,3,3> &_p);//from DCM
      Quaternion(const Eigen::Matrix<double, 4, 1> &_quat);
      Quaternion(const Quaternion &other);
      void setQuaternion(const double &_x, const double &_y, const double &_z, const double &_w);
      auto getQuaternion() const -> const Eigen::Matrix<double,4,1>&;

      auto x() const ->  double ;
      void setX(const double &_x);
      auto y() const -> double;
      void setY(const double &_y);
      auto z() const-> double;
      void setZ(const double &_z);
      auto w() const -> double;
      void setW(const double &_w);

      auto Xi() -> Eigen::Matrix<double, 4, 3>;
      auto Psi() -> Eigen::Matrix<double, 4, 3>;

      Quaternion& operator=(const Quaternion & _other);
      Quaternion& operator*(const Quaternion & _other);
      friend std::ostream& operator<<(std::ostream& os,const Quaternion & _inst);
      ~Quaternion() {};
    private:
      Eigen::Matrix<double,4,1> m_q; //!< this matrix store the quaternion value in the "xyzw" order

    };

};


#endif //REEF_MSGS_Quaternion_H