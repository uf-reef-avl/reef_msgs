//
// Created by prashant on 2/28/19.
//

#include "../include/reef_msgs/matrix_operation.h"
// TODO fix the load transform to make it work in ros2
namespace reef_msgs
{
bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Affine3d &out)
{
    node.declare_parameter(ns+".tx");
    node.declare_parameter(ns+".ty");
    node.declare_parameter(ns+".tz");
    node.declare_parameter(ns+".qx");
    node.declare_parameter(ns+".qy");
    node.declare_parameter(ns+".qz");
    node.declare_parameter(ns+".qw");
    return loadTransform(node,ns,out);
}

bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Affine3d &out)
{
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
    if( node.get_parameter(ns+".tx", tx) &&
        node.get_parameter(ns+".ty",ty) &&
        node.get_parameter(ns+".tz",tz) &&
        node.get_parameter(ns+".qx",qx) &&
        node.get_parameter(ns+".qy",qy) &&
        node.get_parameter(ns+".qz",qz) &&
        node.get_parameter(ns+".qw",qw))
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

bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Matrix4d &out)
{
    node.declare_parameter(ns+".tx");
    node.declare_parameter(ns+".ty");
    node.declare_parameter(ns+".tz");
    node.declare_parameter(ns+".qx");
    node.declare_parameter(ns+".qy");
    node.declare_parameter(ns+".qz");
    node.declare_parameter(ns+".qw");
    return loadTransform(node,ns,out);
}

bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Matrix4d &out)
{
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
    if( node.get_parameter(ns+".tx", tx) &&
        node.get_parameter(ns+".ty",ty) &&
        node.get_parameter(ns+".tz",tz) &&
        node.get_parameter(ns+".qx",qx) &&
        node.get_parameter(ns+".qy",qy) &&
        node.get_parameter(ns+".qz",qz) &&
        node.get_parameter(ns+".qw",qw))
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

bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat)
    {
        node.declare_parameter(ns+".tx");
        node.declare_parameter(ns+".ty");
        node.declare_parameter(ns+".tz");
        node.declare_parameter(ns+".qx");
        node.declare_parameter(ns+".qy");
        node.declare_parameter(ns+".qz");
        node.declare_parameter(ns+".qw");
        return loadTransform(node,ns,out_vec,out_quat);
    }
bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat)
{
  rclcpp::Parameter qx, qy, qz, qw, tx, ty, tz;
  if( node.get_parameter(ns+".tx", tx) &&
          node.get_parameter(ns+".ty",ty) &&
          node.get_parameter(ns+".tz",tz) &&
          node.get_parameter(ns+".qx",qx) &&
          node.get_parameter(ns+".qy",qy) &&
          node.get_parameter(ns+".qz",qz) &&
          node.get_parameter(ns+".qw",qw))
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