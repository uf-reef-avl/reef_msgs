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
  Eigen::Vector3d translation;
  if( nh.getParam("tx", translation(0)) &&
      nh.getParam("ty", translation(1)) &&
      nh.getParam("tz", translation(2)) &&
      nh.getParam("qx", rotation.x()) &&
      nh.getParam("qy", rotation.y()) &&
      nh.getParam("qz", rotation.z()) &&
      nh.getParam("qw", rotation.w()))
  {
    out.translation() = translation;
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
  Eigen::Vector3d translation;
  if( nh.getParam("tx", translation(0)) &&
      nh.getParam("ty", translation(1)) &&
      nh.getParam("tz", translation(2)) &&
      nh.getParam("qx", rotation.x()) &&
      nh.getParam("qy", rotation.y()) &&
      nh.getParam("qz", rotation.z()) &&
      nh.getParam("qw", rotation.w()))
  {
    out.block<3,3>(0,0) = rotation.toRotationMatrix();
    out.block<3,1>(0,3) = translation;
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

bool matrixToVector(const Eigen::MatrixXd &mat, std::vector<double> &vec)
{
  std::vector<double> vec2(mat.data(), mat.data() + mat.rows() * mat.cols());
  vec = vec2;
}

bool loadTransform(std::string ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat)
{
  ros::NodeHandle nh(ns);
  double qx, qy, qz, qw, tx, ty, tz;
  if( nh.getParam("tx", out_vec(0)) &&
      nh.getParam("ty", out_vec(1)) &&
      nh.getParam("tz", out_vec(2)) &&
      nh.getParam("qx", out_quat.x()) &&
      nh.getParam("qy", out_quat.y()) &&
      nh.getParam("qz", out_quat.z()) &&
      nh.getParam("qw", out_quat.w()))
  {
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