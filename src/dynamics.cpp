//
// Created by prashant on 2/28/19.
//

#include "../include/reef_msgs/dynamics.h"
namespace reef_msgs
{

void roll_pitch_yaw_from_quaternion(Eigen::Matrix<double,4,1>q, double &phi, double &theta, double &psi)
{
  double ex = q(0,0);
  double ey = q(1,0);
  double ez = q(2,0);
  double eo = q(3,0);
  phi   = atan2(2.0*eo*ex + 2.0*ey*ez, eo*eo + ez*ez - ex*ex - ey*ey);
  theta = asin(2.0*eo*ey - 2.0*ex*ez);
  psi   = atan2(2.0*eo*ez + 2.0*ex*ey, eo*eo + ex*ex - ey*ey - ez*ez);
}
void roll_pitch_yaw_from_quaternion(Eigen::Quaterniond q, double &phi, double &theta, double &psi)
{
  Eigen::Matrix<double, 4,1> q_mat(q.x(), q.y(), q.z(), q.w());
  roll_pitch_yaw_from_quaternion(q_mat, phi, theta, psi);
}
void roll_pitch_yaw_from_quaternion(geometry_msgs::Quaternion q, double &phi, double &theta, double &psi)
{
  Eigen::Matrix<double, 4,1> q_mat(q.x, q.y, q.z, q.w);
  roll_pitch_yaw_from_quaternion(q_mat, phi, theta, psi);
}

Eigen::Matrix<double,4,1> quaternion_from_roll_pitch_yaw(double phi, double theta, double psi)
{
  double sp = sin(phi/2);
  double cp = cos(phi/2);
  double st = sin(theta/2);
  double ct = cos(theta/2);
  double ss = sin(psi/2);
  double cs = cos(psi/2);

  Eigen::Matrix<double,4,1> quat_return;

  quat_return <<  cp*ct*cs + sp*st*ss, \
                  sp*ct*cs - cp*st*ss , \
                  cp*st*cs + sp*ct*ss, \
                  cp*ct*ss - sp*st*cs;
  return quat_return;

}
void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, Eigen::Quaterniond& q)
{
  Eigen::Matrix<double,4,1> quat_mat = quaternion_from_roll_pitch_yaw(phi,  theta,  psi);

  q.w() = quat_mat(0);
  q.x() = quat_mat(1);
  q.y() = quat_mat(2);
  q.z() = quat_mat(3);
}
void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, geometry_msgs::Quaternion& q)
{
  Eigen::Matrix<double,4,1> quat_mat = quaternion_from_roll_pitch_yaw(phi,  theta,  psi);

  q.w = quat_mat(0);
  q.x = quat_mat(1);
  q.y = quat_mat(2);
  q.z = quat_mat(3);
}

Eigen::Matrix3d quaternion_to_rotation_321(Eigen::Quaterniond q)
{
  Eigen::Matrix3d I;
  I.setIdentity();

  Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q.x(), q.y(), q.z()));
  Eigen::Matrix3d rotation_matrix = I - 2 * q.w() * skew_mat + 2 * skew_mat * skew_mat;
  return rotation_matrix;
}
Eigen::Matrix3d quaternion_to_rotation_321(geometry_msgs::Quaternion q)
{
  Eigen::Quaterniond q_temp;
  q_temp.x() = q.x;
  q_temp.y() = q.y;
  q_temp.z() = q.z;
  q_temp.w() = q.w;

  return quaternion_to_rotation_321(q_temp);
}

Eigen::Matrix3d roll_pitch_yaw_to_rotation_321(double yaw, double roll, double pitch)
{
  Eigen::Quaterniond q;
  quaternion_from_roll_pitch_yaw(pitch, roll, yaw, q);
  return quaternion_to_rotation_321(q);
}

void roll_pitch_yaw_from_rotation(Eigen::Matrix3d C, double& yaw, double& roll, double& pitch )
{
  pitch = -asin(C(0,2));
  roll =  atan2(C(0,2),C(2,2));
  yaw =  atan2(C(0,1),C(0,0));
}

void yaw_from_rotation(Eigen::Matrix3d C, double& yaw)
{
  yaw =  atan2(C(0,1),C(0,0));
}



}