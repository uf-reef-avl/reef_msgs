//
// Created by prashant on 2/28/19.
//
#pragma once
#ifndef PROJECT_MATRIX_OPERATION_H
#define PROJECT_MATRIX_OPERATION_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

namespace reef_msgs
{
template <class Derived>
bool vectorToMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
{
  assertm(vec.size() == mat.rows()*mat.cols(),"Vector size is not equal to matrix size");
  if(vec.size() != mat.rows()*mat.cols())
    return false;
  for(unsigned i=0; i < mat.rows(); i++)
  {
    for(unsigned j=0; j < mat.cols(); j++)
    {
      mat(i,j) = vec[mat.cols()*i+j];
    }
  }
  return true;
}

template <class Derived>
void vectorToDiagMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
{
    assertm(vec.size() == mat.rows(),"Vector size is not equal to matrix row size");
  mat.setZero();
  for(unsigned i=0; i < mat.rows(); i++)
  {
    mat(i,i) = vec[i];
  }
}

template <class Derived>
void importMatrixFromParamServer(const rclcpp::Node &_node, Eigen::MatrixBase<Derived>& mat, std::string param)
{
  std::vector<double> vec;

  rclcpp::Parameter vecParam;
  if(!_node.get_parameter(param, vecParam))
  {
      RCLCPP_WARN_STREAM(_node.get_logger(),"Could not find"<< _node.get_name()<<"/"<<param<<" on server. Zeros!");
    mat.setZero();
    return;
  }
  vec =  vecParam.as_double_array();
  if(vec.size() == mat.rows()*mat.cols())
  {
    //ROS_WARN("Reading %s/%s from server. (Full)",nh.getNamespace().c_str(),param.c_str());
    vectorToMatrix(mat,vec);
  }
  else if(vec.size() == mat.rows())
  {
    //ROS_WARN("Reading %s/%s from server. (Diagonal)",nh.getNamespace().c_str(),param.c_str());
    vectorToDiagMatrix(mat,vec);
  }
  else
  {
      RCLCPP_ERROR_STREAM(_node.get_logger(),"Param"<< _node.get_name()<<"/"<<param<<" is the wrong size. "<< vec.size() << " not "<<mat.rows() << " or "<< mat.rows()*mat.cols());
  }
}

template <class Derived, std::size_t N>
bool matrixToArray(const Eigen::MatrixBase<Derived> &mat, std::array<double,N> &vec)
{
    assertm(vec.size() == mat.rows()*mat.cols(),"Vector size is not equal to matrix size");
  if(vec.size() != mat.rows()*mat.cols())
    return false;
  for(size_t i=0; i < mat.rows(); i++)
  {
    for(size_t j=0; j < mat.cols(); j++)
    {
      vec[mat.cols()*i+j] = mat(i,j);
    }
  }
  return true;
}

template <class Derived>
void verifyDimensions(const Eigen::MatrixBase<Derived> &mat, std::string name, int rows, int cols)
{
    assertm( mat.rows() == rows &&  mat.cols() == cols ,"Invalid matrix dimensions");
}

inline Eigen::Affine3d tf2Eigen(const tf2::Stamped<tf2::Transform> transform)
{
    Eigen::Affine3d temp;
    temp.translation() << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    Eigen::Quaterniond quat_temp(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
    temp.linear() = quat_temp.toRotationMatrix();

    return temp;
}


bool matrixToVector(const Eigen::MatrixXd &mat, std::vector<double> &vec);
bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Affine3d &out);
bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Matrix4d &out);
bool declareAndLoadTransform(rclcpp::Node &node,const std::string & ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat);
bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Affine3d &out);
bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Matrix4d &out);
bool loadTransform(const rclcpp::Node &node,const std::string & ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat);
}


#endif //PROJECT_MATRIX_OPERATION_H
