//
// Created by paul on 10/1/20.
//
#pragma once
#ifndef REEF_MSGS_REEFOBJECTCONVERSION_H
#define REEF_MSGS_REEFOBJECTCONVERSION_H

#include "ReefQuat.h"
#include <boost/any.hpp>
#include <boost/variant.hpp>
#include <boost/type_index.hpp>

typedef boost::variant<geometry_msgs::Quaternion,
        geometry_msgs::QuaternionStamped,
        Eigen::Matrix<double,3, 1>> boostVariantQuatInputOutput;


namespace reef_msgs{
    template<typename Derived>
    void HandleEigenMatrixQuat(const Eigen::MatrixBase<Derived> matrix ,double &x,double &y,double &z,double &w)
    {
        x = matrix.coeff(0,0);
        y = matrix.coeff(1,0);
        z = matrix.coeff(2,0);
        w = matrix.coeff(3,0);
    }

    template<class T>
    reef_msgs::ReefQuat fromAnyTypeToReefQuat(const boostVariantQuatInputOutput &_msgs ){
        std::cout << boost::typeindex::type_id_with_cvr<T&>().pretty_name() << std::endl;
        std::cout <<boost::typeindex::type_id_with_cvr<Eigen::Matrix<double,3, 1>&>().pretty_name()<< std::endl;
        double x,y,z,w;
        if(boost::typeindex::type_id_with_cvr<T&>().pretty_name() == boost::typeindex::type_id_with_cvr<geometry_msgs::Quaternion&>().pretty_name()) {
            std::cout<< "je suis dans la boucle dpd";
            x = boost::get<const geometry_msgs::Quaternion&>(_msgs).x;
            y = boost::get<const geometry_msgs::Quaternion&>(_msgs).y;
            z = boost::get<const geometry_msgs::Quaternion&>(_msgs).z;
            w = boost::get<const geometry_msgs::Quaternion&>(_msgs).w;
        } else if(boost::typeindex::type_id_with_cvr<T&>().pretty_name() == boost::typeindex::type_id_with_cvr<geometry_msgs::QuaternionStamped&>().pretty_name()) {
            x = boost::get<const geometry_msgs::QuaternionStamped&>(_msgs).quaternion.x;
            y = boost::get<const geometry_msgs::QuaternionStamped&>(_msgs).quaternion.y;
            z = boost::get<const geometry_msgs::QuaternionStamped&>(_msgs).quaternion.z;
            w = boost::get<const geometry_msgs::QuaternionStamped&>(_msgs).quaternion.w;
        }else if(boost::typeindex::type_id_with_cvr<T&>().pretty_name() == boost::typeindex::type_id_with_cvr<Eigen::Matrix<double,3, 1>&>().pretty_name()) {
            std::cout<< "un peu nla mùerdum avec la parti";
            x = boost::get<const Eigen::Matrix<double,3, 1>&>(_msgs).coeff(0,0);
            y = boost::get<const Eigen::Matrix<double,3, 1>&>(_msgs).coeff(1,0);
            z = boost::get<const Eigen::Matrix<double,3, 1>&>(_msgs).coeff(2,0);
            w = boost::get<const Eigen::Matrix<double,3, 1>&>(_msgs).coeff(3,0);
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::ReefQuat(x,y,z,w);
    }

    template<class T>
    T fromReefQuatToAnyType(const reef_msgs::ReefQuat &_reefQuat){
        std::cout << boost::typeindex::type_id_with_cvr<T&>().pretty_name() << std::endl;
        geometry_msgs::Twist v;
        return v;
    }
}

#endif //REEF_MSGS_REEFOBJECTCONVERSION_H
