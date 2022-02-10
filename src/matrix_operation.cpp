//
// Created by prashant on 2/28/19.
//

#include "../include/reef_msgs/matrix_operation.h"
// TODO fix the load transform to make it work in ros2
namespace reef_msgs
{
bool loadTransform(const rclcpp::Node &node, Eigen::Affine3d &out)
{
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
    if( node.get_parameter("tx", tx) &&
        node.get_parameter("ty",ty) &&
        node.get_parameter("tz",tz) &&
        node.get_parameter("qx",qx) &&
        node.get_parameter("qy",qy) &&
        node.get_parameter("qz",qz) &&
        node.get_parameter("qw",qw))
    {
    translation(0) = tx.as_double();
    translation(1) = ty.as_double();
    translation(2) = tz.as_double();
    rotation.x() = qx.as_double();
    rotation.y() = qy.as_double();
    rotation.z() = qz.as_double();
    rotation.w() = qw.as_double();
    out.translation() = translation;
    out.linear() = rotation.toRotationMatrix();
    return true;
  }
  else
  {
      RCLCPP_ERROR_STREAM(node.get_logger(),"Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw" << node.get_name());
    out.linear() = Eigen::Matrix3d::Identity();
    return false;
  }
}

bool loadTransform(const rclcpp::Node &node, Eigen::Matrix4d &out)
{
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
    if( node.get_parameter("tx", tx) &&
        node.get_parameter("ty",ty) &&
        node.get_parameter("tz",tz) &&
        node.get_parameter("qx",qx) &&
        node.get_parameter("qy",qy) &&
        node.get_parameter("qz",qz) &&
        node.get_parameter("qw",qw))
  {
      translation(0) = tx.as_double();
      translation(1) = ty.as_double();
      translation(2) = tz.as_double();
      rotation.x() = qx.as_double();
      rotation.y() = qy.as_double();
      rotation.z() = qz.as_double();
      rotation.w() = qw.as_double();


    out.block<3,3>(0,0) = rotation.toRotationMatrix();
    out.block<3,1>(0,3) = translation;
    out(3,3) = 1;
    return true;
  }
  else
  {
      RCLCPP_ERROR_STREAM(node.get_logger(),"Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw" << node.get_name());
    out.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    return false;
  }
}

bool matrixToVector(const Eigen::MatrixXd &mat, std::vector<double> &vec)
{
  std::vector<double> vec2(mat.data(), mat.data() + mat.rows() * mat.cols());
  vec = vec2;
}
//
bool loadTransform(const rclcpp::Node &node, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat)
{
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
  if( node.get_parameter("tx", tx) &&
          node.get_parameter("ty",ty) &&
          node.get_parameter("tz",tz) &&
          node.get_parameter("qx",qx) &&
          node.get_parameter("qy",qy) &&
          node.get_parameter("qz",qz) &&
          node.get_parameter("qw",qw))
  {
      out_vec(0) = tx.as_double();
       out_vec(1) = ty.as_double();
       out_vec(2) = tz.as_double();
       out_quat.x() = qx.as_double();
       out_quat.y() = qy.as_double();
       out_quat.z() = qz.as_double();
       out_quat.w() = qw.as_double();
    return true;
  }
  else
  {
      RCLCPP_ERROR_STREAM(node.get_logger(),"Expected transform not found! Set %s/tx:ty:tz:qx:qy:qz:qw" << node.get_name());
    out_quat.w() = 0;
    out_quat.x() = 0;
    out_quat.y() = 0;
    out_quat.z() = 1;

    return false;
  }
}

}