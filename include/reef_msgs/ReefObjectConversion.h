//
// Created by paul on 10/1/20.
//
#pragma once
#ifndef REEF_MSGS_REEFOBJECTCONVERSION_H
#define REEF_MSGS_REEFOBJECTCONVERSION_H

#include "ReefQuat.h"
#include <iostream>

namespace reef_msgs{

    template<class T>
    reef_msgs::ReefQuat fromAnyTypeToReefQuat(const T &_msgs ){
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
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::ReefQuat(x,y,z,w);
    }

    template<class T>
    T fromReefQuatToAnyType(const reef_msgs::ReefQuat &_reefQuat){
        //std::cout << boost::typeindex::type_id_with_cvr<T>::value.pretty_name() << std::endl;
        if constexpr(std::is_same<geometry_msgs::Quaternion, T>::value) {
            geometry_msgs::Quaternion q;
            q.x = _reefQuat.x();
            q.y = _reefQuat.y();
            q.z = _reefQuat.z();
            q.w = _reefQuat.w();
            return q;
        } else if constexpr(std::is_same<geometry_msgs::QuaternionStamped, T>::value) {
            geometry_msgs::QuaternionStamped q;
            q.quaternion.x = _reefQuat.x();
            q.quaternion.y = _reefQuat.y();
            q.quaternion.z = _reefQuat.z();
            q.quaternion.w = _reefQuat.w();
            return q;
        }else if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
                 || (std::is_same<Eigen::Vector4d , T>::value)) {
            Eigen::Matrix<double,4, 1> q;
            q << _reefQuat.x(), _reefQuat.y(), _reefQuat.z(), _reefQuat.w();
            return q;
        }else if constexpr((std::is_same<Eigen::Matrix<float,4, 1>, T>::value)
                 || (std::is_same<Eigen::Vector4f, T>::value)) {
            Eigen::Matrix<float,4, 1> q;
            q << _reefQuat.x(), _reefQuat.y(), _reefQuat.z(), _reefQuat.w();
            return q;
        }else if constexpr(std::is_same<std::vector<double>, T>::value) {
            std::vector<double> q;
            q.push_back(_reefQuat.x());
            q.push_back(_reefQuat.y());
            q.push_back(_reefQuat.z());
            q.push_back(_reefQuat.w());
            return q;
        }else if constexpr(std::is_same<std::vector<float>, T>::value) {
            std::vector<float> q;
            q.push_back(_reefQuat.x());
            q.push_back(_reefQuat.y());
            q.push_back(_reefQuat.z());
            q.push_back(_reefQuat.w());
            return q;
        } else{
            throw "This type isn't defined";
        }
    }
}

#endif //REEF_MSGS_REEFOBJECTCONVERSION_H
