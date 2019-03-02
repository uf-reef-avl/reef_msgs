//
// Created by prashant on 2/28/19.
//

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

Eigen::Matrix3d quaternion_to_rotation_321(Eigen::Quaterniond q);
Eigen::Matrix3d quaternion_to_rotation_321(geometry_msgs::Quaternion q);

Eigen::Matrix3d roll_pitch_yaw_to_rotation_321(double yaw, double roll, double pitch);

void roll_pitch_yaw_from_rotation(Eigen::Matrix3d C, double& yaw, double& roll, double& pitch );
void yaw_from_rotation(Eigen::Matrix3d C, double& yaw);

inline double getYaw(Eigen::Quaterniond q)
  {
    return std::atan2(2.0*(q.w()*q.z()+q.x()*q.y()), 1.0-2.0*(q.y()*q.y() + q.z()*q.z()));
  }

inline double getYaw(geometry_msgs::Quaternion q)
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

}
#endif //PROJECT_DYNAMICS_H
