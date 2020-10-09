//
// Created by paul on 10/1/20.
//
#pragma once
#include <iostream>
#include "Quaternion.h"
#include "RodriguezParameter.h"
#include "RotationMatrix.h"
#include "EulerAngle.h"
#include "DCM.h"
#include "AxisAngle.h"

namespace reef_msgs{
    template<class T>
    reef_msgs::Quaternion fromAnyTypeToQuaternion(const T &_msgs ){
        double x,y,z,w;
        if constexpr(std::is_same<geometry_msgs::Quaternion, T>::value) {
            x = _msgs.x;
            y = _msgs.y;
            z = _msgs.z;
            w = _msgs.w;
        } else if constexpr(std::is_same<geometry_msgs::QuaternionStamped, T>::value) {
            x = _msgs.quaternion.x;
            y = _msgs.quaternion.y;
            z = _msgs.quaternion.z;
            w = _msgs.quaternion.w;
        } else if constexpr((std::is_same<Eigen::Quaterniond, T>::value)
                            || (std::is_same<Eigen::Quaternionf, T>::value)) {
            x = _msgs.x();
            y = _msgs.y();
            z = _msgs.z();
            w = _msgs.w();
        }else if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4d , T>::value)
                           || (std::is_same<Eigen::Matrix<float,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4f, T>::value) ) {
            x = _msgs.coeff(0,0);
            y = _msgs.coeff(1,0);
            z = _msgs.coeff(2,0);
            w = _msgs.coeff(3,0);
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            x = _msgs.at(0);
            y = _msgs.at(1);
            z = _msgs.at(2);
            w = _msgs.at(3);
        }else if constexpr(std::is_same<tf2::Quaternion, T>::value) {
            x = _msgs.x();
            y = _msgs.y();
            z = _msgs.w();
            w = _msgs.z();
        }else if constexpr(std::is_same<geometry_msgs::Transform, T>::value) {
            x = _msgs.rotation.x;
            y = _msgs.rotation.y;
            z = _msgs.rotation.z;
            w = _msgs.rotation.w;
        }else if constexpr(std::is_same<geometry_msgs::TransformStamped, T>::value) {
            x = _msgs.transform.rotation.x;
            y = _msgs.transform.rotation.y;
            z = _msgs.transform.rotation.z;
            w = _msgs.transform.rotation.w;
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::Quaternion(x,y,z,w);
    }
    template<class T>
    reef_msgs::AxisAngle fromAnyTypeToAxisAngle(const T &_msgs ){
        double x,y,z,angle;

        if constexpr(std::is_same<geometry_msgs::QuaternionStamped, T>::value) {
            x = _msgs.quaternion.x;
            y = _msgs.quaternion.y;
            z = _msgs.quaternion.z;
            angle = _msgs.quaternion.w;
        }else if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4d , T>::value)
                           || (std::is_same<Eigen::Matrix<float,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4f, T>::value) ) {
            x = _msgs.coeff(0,0);
            y = _msgs.coeff(1,0);
            z = _msgs.coeff(2,0);
            angle = _msgs.coeff(3,0);
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            x = _msgs.at(0);
            y = _msgs.at(1);
            z = _msgs.at(2);
            angle = _msgs.at(3);
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::AxisAngle(x,y,z,angle);
    }

    template<class T>
    T fromQuaternionToAnyType(const reef_msgs::Quaternion &_Quaternion){
        //std::cout << boost::typeindex::type_id_with_cvr<T>::value.pretty_name() << std::endl;
        if constexpr(std::is_same<geometry_msgs::Quaternion, T>::value) {
            geometry_msgs::Quaternion q;
            q.x = _Quaternion.x();
            q.y = _Quaternion.y();
            q.z = _Quaternion.z();
            q.w = _Quaternion.w();
            return q;
        } else if constexpr(std::is_same<geometry_msgs::QuaternionStamped, T>::value) {
            geometry_msgs::QuaternionStamped q;
            q.quaternion.x = _Quaternion.x();
            q.quaternion.y = _Quaternion.y();
            q.quaternion.z = _Quaternion.z();
            q.quaternion.w = _Quaternion.w();
            return q;
        } else if constexpr(std::is_same<Eigen::Quaterniond, T>::value) {
            Eigen::Quaterniond q = Eigen::Quaterniond(_Quaternion.w(), _Quaternion.x(), _Quaternion.y(), _Quaternion.z());
            return q;
        } else if constexpr(std::is_same<Eigen::Quaternionf, T>::value) {
            Eigen::Quaterniond q = Eigen::Quaterniond(_Quaternion.w(), _Quaternion.x(), _Quaternion.y(), _Quaternion.z());
            return q;
        }else if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4d , T>::value)) {
            Eigen::Matrix<double,4, 1> q;
            q << _Quaternion.x(), _Quaternion.y(), _Quaternion.z(), _Quaternion.w();
            return q;
        }else if constexpr((std::is_same<Eigen::Matrix<float,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4f, T>::value)) {
            Eigen::Matrix<float,4, 1> q;
            q << _Quaternion.x(), _Quaternion.y(), _Quaternion.z(), _Quaternion.w();
            return q;
        }else if constexpr(std::is_same<std::vector<double>, T>::value) {
            std::vector<double> q;
            q.push_back(_Quaternion.x());
            q.push_back(_Quaternion.y());
            q.push_back(_Quaternion.z());
            q.push_back(_Quaternion.w());
            return q;
        }else if constexpr(std::is_same<std::vector<float>, T>::value) {
            std::vector<float> q;
            q.push_back(_Quaternion.x());
            q.push_back(_Quaternion.y());
            q.push_back(_Quaternion.z());
            q.push_back(_Quaternion.w());
            return q;
        }else if constexpr(std::is_same<tf::Quaternion, T>::value) {
            tf::Quaternion q = tf::Quaternion(_Quaternion.x(),_Quaternion.y(),_Quaternion.z(),_Quaternion.w());
            return q;
        }else if constexpr(std::is_same<geometry_msgs::Transform, T>::value) {
            geometry_msgs::Transform q;
            q.rotation.x = _Quaternion.x();
            q.rotation.y = _Quaternion.y();
            q.rotation.z = _Quaternion.z();
            q.rotation.w = _Quaternion.w();
            return q;
        }else if constexpr(std::is_same<geometry_msgs::TransformStamped, T>::value) {
            geometry_msgs::TransformStamped q;
            q.transform.rotation.x = _Quaternion.x();
            q.transform.rotation.y = _Quaternion.y();
            q.transform.rotation.z = _Quaternion.z();
            q.transform.rotation.w = _Quaternion.w();
            return q;
        }else{
                throw "This type isn't defined";
        }
    }
    template<class T>
    T fromAxisAngleToAnyType(const reef_msgs::AxisAngle &_axisAngle){

        if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4d , T>::value)) {
            Eigen::Matrix<double,4, 1> q;
            q << _axisAngle.x(), _axisAngle.y(), _axisAngle.z(), _axisAngle.angle();
            return q;
        }else if constexpr((std::is_same<Eigen::Matrix<float,4, 1>, T>::value)
                           || (std::is_same<Eigen::Vector4f, T>::value)) {
            Eigen::Matrix<float,4, 1> q;
            q << _axisAngle.x(), _axisAngle.y(), _axisAngle.z(), _axisAngle.angle();
            return q;
        }else if constexpr(std::is_same<std::vector<double>, T>::value) {
            std::vector<double> q;
            q.push_back(_axisAngle.x());
            q.push_back(_axisAngle.y());
            q.push_back(_axisAngle.z());
            q.push_back(_axisAngle.angle());
            return q;
        }else if constexpr(std::is_same<std::vector<float>, T>::value) {
            std::vector<float> q;
            q.push_back(_axisAngle.x());
            q.push_back(_axisAngle.y());
            q.push_back(_axisAngle.z());
            q.push_back(_axisAngle.angle());
            return q;
        } else{
            throw "This type isn't defined";
        }
    }
}


