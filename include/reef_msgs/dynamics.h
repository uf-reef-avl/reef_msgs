//
// Created by prashant on 2/28/19.
//
#pragma once
#ifndef PROJECT_DYNAMICS_H
#define PROJECT_DYNAMICS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

namespace reef_msgs
{

void roll_pitch_yaw_from_quaternion(Eigen::Matrix<double,4,1>q, double &phi, double &theta, double &psi);
void roll_pitch_yaw_from_quaternion(Eigen::Quaterniond q, double &phi, double &theta, double &psi);
void roll_pitch_yaw_from_quaternion(geometry_msgs::Quaternion q, double &phi, double &theta, double &psi);

Eigen::Matrix<double,4,1> quaternion_from_roll_pitch_yaw(double phi, double theta, double psi);
void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, Eigen::Quaterniond& q);
void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, geometry_msgs::Quaternion& q);

Eigen::Matrix3d quaternion_to_rotation(Eigen::Quaterniond q);
Eigen::Matrix3d quaternion_to_rotation(geometry_msgs::Quaternion q);

Eigen::Matrix3d roll_pitch_yaw_to_rotation_321(double roll, double pitch, double yaw);
Eigen::Matrix3d DCM_from_Euler321(const Eigen::Vector3d rpy);
Eigen::Quaterniond DCM2quat(const Eigen::Matrix3d C) ;
void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, double& roll, double& pitch, double& yaw );
void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, Eigen::Vector3d &rpy);

Eigen::Matrix4d Omega(Eigen::Vector3d pqr);

Eigen::Matrix<double,4,1> quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q);
Eigen::Quaterniond quaternionMultiplication(Eigen::Quaterniond p , Eigen::Quaterniond q);

Eigen::Quaterniond vector2quat(Eigen::Matrix<double, 4,1> mat_in);
Eigen::Matrix<double, 4,1> quat2vector(Eigen::Quaterniond q_in);

Eigen::Affine3d convertNWU2NED(const Eigen::Affine3d& nwu);

inline Eigen::Vector3d unSkew(const Eigen::Matrix3d& skew_matrix){
    Eigen::Vector3d vec;
    vec << -skew_matrix(1,2) , skew_matrix(0,2), -skew_matrix(0,1);
    return vec;
}

inline void get_yaw(Eigen::Matrix3d C, double& yaw){
  yaw =  atan2(C(0,1),C(0,0));
}

inline double get_yaw(Eigen::Quaterniond q)
  {
    return std::atan2(2.0*(q.w()*q.z()+q.x()*q.y()), 1.0-2.0*(q.y()*q.y() + q.z()*q.z()));
  }

inline double get_yaw(geometry_msgs::Quaternion q)
{
  return std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z));
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d skew;
  skew << 0, -vec(2),  vec(1), \
          vec(2),       0, -vec(0), \
          -vec(1),  vec(0),       0;
  return skew;
}

inline double pi2pi(double angle)
{
    double return_angle;
    if(std::abs(angle - M_PI ) > 0.001)
    {
        if(angle > M_PI)
            return_angle = angle - 2 * M_PI;
        if(angle < -M_PI)
            return_angle = angle + 2 * M_PI;
    } else
        return_angle = angle;
    return return_angle;
}

//inline Eigen::Quaterniond rodrigues2quaternion(double rx, double ry, double rz){
//    Eigen::Quaterniond quat;
//    quat.w() = 1/sqrt(1+rx*rx);
//    quat.x() = rx*rx;
//    quat.y() = ry*rx;
//    quat.z() = rz*rx;
//    return quat;
//}

inline Eigen::Affine3d NWU2NED_Affine(Eigen::Affine3d& in){
    Eigen::Affine3d out;

    Eigen::Quaterniond q(in.linear());
    Eigen::Quaterniond q_temp(q.w(), q.x(), -q.y(), -q.z());

    out.linear() = q_temp.toRotationMatrix();
    out.translation() << in.translation()(0), -in.translation()(1), -in.translation()(2);

//    Eigen::Quaterniond q(0,1,0,0);
//    out.translation() = q.toRotationMatrix() * in.translation();
//    out.linear() = q.toRotationMatrix() * in.linear();
    return out;
}


}
#endif //PROJECT_DYNAMICS_H
