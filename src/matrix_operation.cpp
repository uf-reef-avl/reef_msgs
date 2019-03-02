//
// Created by prashant on 2/28/19.
//

#include "../include/reef_msgs/matrix_operation.h"
#include <ros/ros.h>

namespace reef_msgs
{
bool loadTransform(std::string ns, Eigen::Affine3d &out)
{
  ros::NodeHandle nh(ns);
  Eigen::Quaterniond rotation;
  double qx, qy, qz, qw, tx, ty, tz;
  if( nh.getParam("tx", tx) &&
      nh.getParam("ty", ty) &&
      nh.getParam("tz", tz) &&
      nh.getParam("qx", qx) &&
      nh.getParam("qy", qy) &&
      nh.getParam("qz", qz) &&
      nh.getParam("qw", qw))
  {
    rotation.w() = qw;
    rotation.x() = qx;
    rotation.y() = qy;
    rotation.z() = qz;
    out.translation() = Eigen::Vector3d(tx,ty,tz);
    out.linear() = rotation.toRotationMatrix();
    return true;
  }
  else
  {
    ROS_ERROR("Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw",nh.getNamespace().c_str());
    out.linear() = Eigen::Matrix3d::Identity();
    return false;
  }
}

bool loadTransform(std::string ns, Eigen::Matrix4d &out)
{
  ros::NodeHandle nh(ns);
  Eigen::Quaterniond rotation;
  double qx, qy, qz, qw, tx, ty, tz;
  if( nh.getParam("tx", tx) &&
      nh.getParam("ty", ty) &&
      nh.getParam("tz", tz) &&
      nh.getParam("qx", qx) &&
      nh.getParam("qy", qy) &&
      nh.getParam("qz", qz) &&
      nh.getParam("qw", qw))
  {
    rotation.w() = qw;
    rotation.x() = qx;
    rotation.y() = qy;
    rotation.z() = qz;
    out.block<3,3>(0,0) = rotation.toRotationMatrix();
    out.block<3,1>(0,3) = Eigen::Vector3d(tx,ty,tz);
    out(3,3) = 1;
    return true;
  }
  else
  {
    ROS_ERROR("Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw",nh.getNamespace().c_str());
    out.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    return false;
  }
}



Eigen::Affine3d convertNWU2NED(const Eigen::Affine3d& nwu)
{
  Eigen::Quaterniond q(0,1,0,0);
  Eigen::Affine3d nwu_to_ned_dcm;
  nwu_to_ned_dcm.linear() = q.toRotationMatrix();
  Eigen::Affine3d ned = nwu_to_ned_dcm.inverse() *  nwu * nwu_to_ned_dcm;
  return ned;
}

bool matrixToVector(const Eigen::MatrixXd &mat, std::vector<double> &vec)
{
  std::vector<double> vec2(mat.data(), mat.data() + mat.rows() * mat.cols());
  vec = vec2;
}

bool loadTransform(std::string ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat)
{
  ros::NodeHandle nh(ns);
  double qx, qy, qz, qw, tx, ty, tz;
  if( nh.getParam("tx", tx) &&
      nh.getParam("ty", ty) &&
      nh.getParam("tz", tz) &&
      nh.getParam("qx", qx) &&
      nh.getParam("qy", qy) &&
      nh.getParam("qz", qz) &&
      nh.getParam("qw", qw))
  {
    out_quat.w() = qw;
    out_quat.x() = qx;
    out_quat.y() = qy;
    out_quat.z() = qz;

    out_vec = Eigen::Vector3d(tx,ty,tz);
    return true;
  }
  else
  {
    ROS_ERROR("Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw",nh.getNamespace().c_str());
    out_quat.w() = 0;
    out_quat.x() = 0;
    out_quat.y() = 0;
    out_quat.z() = 1;

    return false;
  }
}

}