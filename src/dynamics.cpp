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

Eigen::Matrix3d quaternion_to_rotation(Eigen::Quaterniond q)
{
  Eigen::Matrix3d I;
  I.setIdentity();

  Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q.x(), q.y(), q.z()));
  Eigen::Matrix3d rotation_matrix = I - 2 * q.w() * skew_mat + 2 * skew_mat * skew_mat;

  return rotation_matrix;
}

Eigen::Matrix3d quaternion_to_rotation(geometry_msgs::Quaternion q)
{
  Eigen::Quaterniond q_temp;
  q_temp.x() = q.x;
  q_temp.y() = q.y;
  q_temp.z() = q.z;
  q_temp.w() = q.w;

  return quaternion_to_rotation(q_temp);
}

Eigen::Matrix3d roll_pitch_yaw_to_rotation_321(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond q;
  quaternion_from_roll_pitch_yaw(roll,pitch, yaw, q);
  return quaternion_to_rotation(q);
}

void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, double& roll, double& pitch, double& yaw)
{
  roll =  atan2(C(1,2),C(2,2));//roll
  pitch = -asin(C(0,2)); //pitch
  yaw =  atan2(C(0,1),C(0,0)); //yaw
}

void roll_pitch_yaw_from_rotation321(Eigen::Matrix3d C, Eigen::Vector3d &rpy){
  rpy(0) = atan2(C(1,2),C(2,2)); //roll
  rpy(1) = -asin(C(0,2)); // pitch
  rpy(2) = atan2(C(0,1),C(0,0)); // pitch
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
  double pw = p(3,0);

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d p_skew;
  p_skew = skew(p.head(3));

  Eigen::Matrix4d A;
  A.block<3,3>(0,0) = pw * I * p_skew;
  A.block<4,1>(0,3) = p;
  A.block<1,4>(3,0) = -p.transpose();
  A(3,3) = pw;

  return A*q;
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

Eigen::Matrix3d DCM_from_Euler321(const Eigen::Vector3d rpy)
{
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Eigen::Matrix3d C3, C2, C1;

    C3 << cos(yaw), sin(yaw), 0,  -sin(yaw), cos(yaw), 0, 0,0,1;
    C2 << cos(pitch),0, -sin(pitch), 0, 1, 0,sin(pitch), 0,cos(pitch);
    C1 << 1, 0, 0, 0, cos(roll), sin(roll), 0, -sin(roll), cos(roll);

    return C1*C2*C3;
}

    Eigen::Quaterniond DCM2quat(const Eigen::Matrix3d C) {
/*    Use Sheppard's algorithm to convert from direction cosine matrix to
    quaternion. See Hurtado, J.E., Kinematic and Kinetic Principles.
    */
        float gam = C.trace();
        float w2 = (1.0 + gam) / 4.;
        Eigen::Matrix<double,4,1>  q2;
        q2(0) = (1 + 2 * C(0,0) - gam) / 4.;
        q2(1) = (1 + 2 * C(1,1) - gam) / 4.;
        q2(2) = (1 + 2 * C(2,2) - gam) / 4.;
        q2(3) = w2;
        int max_index = q2.maxCoeff();
        Eigen::Matrix<double,4,1>  q;
        q << 0,0,0,0;
        q(max_index) = sqrt(q2(max_index));
        float d = 4. * q(max_index);
        float C11 = C(0,0);
        float C12 = C(0,1);
        float C13 = C(0,2);
        float C21 = C(1,0);
        float C22 = C(1,1);
        float C23 = C(1,2);
        float C31 = C(2,0);
        float C32 = C(2,1);
        float C33 = C(2,2);
        if (max_index == 3){
            q(0) = (C23 - C32) / d;
            q(1) = (C31 - C13) / d;
            q(2) = (C12 - C21) / d;
        }
        else if (max_index == 0) {
            q(3) = (C23 - C32) / d;
            q(1) = (C12 + C21) / d;
            q(2) = (C31 + C13) / d;
        }
        else if (max_index == 1) {
            q(3) = (C31 - C13) / d;
            q(0) = (C12 + C21) / d;
            q(2) = (C23 + C32) / d;
        }
        else if (max_index == 2){
            q(3) = (C12 - C21) / d;
            q(0) = (C31 + C13) / d;
            q(1) = (C23 + C32) / d;
        }
        Eigen::Quaterniond quat;
        quat.coeffs()<<q(0),q(1),q(2),q(3);
        quat.normalize();
        return quat;
    }

}
