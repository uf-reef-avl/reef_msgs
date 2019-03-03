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

Eigen::Matrix4d Omega(Eigen::Vector3d pqr)
{
  Eigen::Matrix4d omega;
  omega.block<3,3>(0,0) = -skew(pqr);
  omega.block<3,1>(0,3) = pqr;
  omega.block<1,3>(3,0) = -pqr.transpose();
  omega(3,3) = 0;
  return omega;
}

Eigen::Matrix<double,4,1> quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q)
{
  double px = p(0,0);
  double py = p(1,0);
  double pz = p(2,0);
  double pw = p(3,0);

  Eigen::Matrix<double,4,4> p_matrix;
  p_matrix << pw, -pz,  py,  px, \
              pz,  pw, -px,  py, \
             -py,  px,  pw,  pz, \
             -px, -py, -pz,  pw;
  return p_matrix*q;
}

Eigen::Quaterniond quaternionMultiplication(Eigen::Quaterniond p , Eigen::Quaterniond q)
{
  Eigen::Matrix<double,4,1> p_temp;
  Eigen::Matrix<double,4,1> q_temp;
  Eigen::Matrix<double,4,1> multiplicant;

  p_temp = quat2vector(p);
  q_temp = quat2vector(q);

  multiplicant = quaternionMultiplication(p_temp,q_temp);

  return vector2quat(multiplicant);

}

Eigen::Quaterniond vector2quat(Eigen::Matrix<double, 4,1> mat_in)
{
  Eigen::Quaterniond q_out;
  q_out.x() = mat_in(0,0);
  q_out.y() = mat_in(1,0);
  q_out.z() = mat_in(2,0);
  q_out.w() = mat_in(3,0);

  return q_out;
}

Eigen::Matrix<double, 4,1> quat2vector(Eigen::Quaterniond q_in)
{
  Eigen::Matrix<double, 4,1> mat_out;
  mat_out(0,0) = q_in.x();
  mat_out(1,0) = q_in.y();
  mat_out(2,0) = q_in.z();
  mat_out(3,0) = q_in.w();

  return mat_out;
}

Eigen::Affine3d convertNWU2NED(const Eigen::Affine3d& nwu)
{
  Eigen::Quaterniond q(0,1,0,0); // w,x,y,z
  Eigen::Affine3d nwu_to_ned_dcm;
  nwu_to_ned_dcm.linear() = q.toRotationMatrix();
  Eigen::Affine3d ned = nwu_to_ned_dcm.inverse() *  nwu * nwu_to_ned_dcm;
  return ned;
}

}