//
// Created by prashant on 2/28/19.
//
#pragma once
#ifndef PROJECT_MATRIX_OPERATION_H
#define PROJECT_MATRIX_OPERATION_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <tf/tf.h>

namespace reef_msgs
{
template <class Derived>
bool vectorToMatrix(Eigen::MatrixBase<Derived>& mat, std::vector<double> vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
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
  ROS_ASSERT(vec.size() == mat.rows());
  mat.setZero();
  for(unsigned i=0; i < mat.rows(); i++)
  {
    mat(i,i) = vec[i];
  }
}

template <class Derived>
void importMatrixFromParamServer(const ros::NodeHandle nh, Eigen::MatrixBase<Derived>& mat, std::string param)
{
  std::vector<double> vec;
  if(!nh.getParam(param, vec))
  {
    ROS_WARN("Could not find %s/%s on server. Zeros!",nh.getNamespace().c_str(),param.c_str());
    mat.setZero();
    return;
  }
  else if(vec.size() == mat.rows()*mat.cols())
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
    ROS_ERROR("Param %s/%s is the wrong size. %f not %f or %f" ,nh.getNamespace().c_str(),param.c_str(),(double) vec.size(),(double) mat.rows(),(double) mat.rows()*mat.cols());
  }
}

template <class Derived, std::size_t N>
bool matrixToArray(const Eigen::MatrixBase<Derived> &mat, boost::array<double,N> &vec)
{
  ROS_ASSERT(vec.size() == mat.rows()*mat.cols());
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
  ROS_ASSERT_MSG( mat.rows() == rows &&  mat.cols() == cols ,
                  "%s is %dx%d. Expecting %dx%d",name.c_str(),(int) mat.rows(), (int) mat.cols(),rows,cols);
}

inline Eigen::Affine3d tf2Eigen(const tf::StampedTransform transform)
{
    Eigen::Affine3d temp;
    temp.translation() << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    Eigen::Quaterniond quat_temp(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
    temp.linear() = quat_temp.toRotationMatrix();

    return temp;
}


bool matrixToVector(const Eigen::MatrixXd &mat, std::vector<double> &vec);
bool loadTransform(std::string ns, Eigen::Affine3d &out);
bool loadTransform(std::string ns, Eigen::Matrix4d &out);
bool loadTransform(std::string ns, Eigen::Vector3d &out_vec, Eigen::Quaterniond &out_quat);
}


#endif //PROJECT_MATRIX_OPERATION_H
