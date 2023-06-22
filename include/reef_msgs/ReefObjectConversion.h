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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace reef_msgs{
    /*!
    @brief Generate a reef_msgs quaternion from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs quaternion. Right now, the type supported are:
     geometry_msgs::Quaternion,
     geometry_msgs::QuaternionStamped,
     Eigen::Quaterniond,
     Eigen::Matrix<double,4, 1>,
     Eigen::Vector4d ,
     Eigen::Matrix<float,4, 1>,
     Eigen::Vector4f,
     std::vector<double>,
     std::vector<float>,
     tf2::Quaternion,
     geometry_msgs::Transform,
     geometry_msgs::TransformStamped,
    @return the reef_mgs quaternion instancy created
    */
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
    /*!
    @brief Generate a reef_msgs axisangle from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs axiangle. Right now, the type supported are:
     Eigen::Matrix<double,4, 1>,
     Eigen::Vector4d ,
     Eigen::Matrix<float,4, 1>,
     Eigen::Vector4f,
     std::vector<double>,
     std::vector<float>,
    @return the reef_mgs axisangle instancy created
    */
    template<class T>
    reef_msgs::AxisAngle fromAnyTypeToAxisAngle(const T &_msgs ){
        double x,y,z,angle;

        if constexpr((std::is_same<Eigen::Matrix<double,4, 1>, T>::value)
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
    /*!
    @brief Generate a reef_msgs DCM from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs DCM. Right now, the type supported are:
     Eigen::Matrix3d,
     Eigen::Matrix3f,
     Eigen::Matrix<double,3,3>,
     Eigen::Matrix<float,3,3>,
    @return the reef_mgs DCM instancy created
    */
    template<class T>
    reef_msgs::DCM fromAnyTypeToDCM(const T &_msgs ){
        Eigen::Matrix3d mat;
        if constexpr((std::is_same<Eigen::Matrix3d, T>::value)
        || (std::is_same<Eigen::Matrix<double,3,3>, T>::value)
           || (std::is_same<Eigen::Matrix3f, T>::value)
              || (std::is_same<Eigen::Matrix<float,3,3>, T>::value)){
            mat << _msgs;
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::DCM(mat);
    }
    /*!
    @brief Generate a reef_msgs euler angle from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs euler angle. Right now, the type supported are:
     geometry_msgs::Vector3,
     geometry_msgs::Vector3Stamped,
     Eigen::Matrix<double,3, 1>,
     Eigen::Matrix<float,3, 1>,
     Eigen::Vector3d,
     Eigen::Vector3f,
     @param[in] _eulerTransformation  the rotation order applied on the euler angle
    @return the reef_mgs euler angle instancy created
    */
    template<class T>
    reef_msgs::EulerAngle fromAnyTypeToEulerAngle(const T &_msgs , const std::string &_eulerTransformation){
        double yaw,pitch,roll;
        if constexpr(std::is_same<geometry_msgs::Vector3 , T>::value){
            yaw = _msgs.x;
            pitch = _msgs.y;
            roll = _msgs.z;
        }else if constexpr(std::is_same<geometry_msgs::Vector3Stamped, T>::value) {
            yaw = _msgs.vector.x;
            pitch = _msgs.vector.y;
            roll = _msgs.vector.z;
        }else if constexpr((std::is_same<Eigen::Matrix<double,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3d , T>::value)
                           || (std::is_same<Eigen::Matrix<float,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3f, T>::value) ) {
            yaw = _msgs.coeff(0,0);
            pitch = _msgs.coeff(1,0);
            roll = _msgs.coeff(2,0);
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            yaw = _msgs.at(0);
            pitch = _msgs.at(1);
            roll = _msgs.at(2);
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::EulerAngle(yaw,pitch,roll,_eulerTransformation);

    }
    /*!
    @brief Generate a reef_msgs rodriguez parameter from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs rodriguez parameter angle. Right now, the type supported are:
     geometry_msgs::Vector3,
     geometry_msgs::Vector3Stamped,
     Eigen::Matrix<double,3, 1>,
     Eigen::Matrix<float,3, 1>,
     Eigen::Vector3d,
     Eigen::Vector3f,
     std::vector<double>,
     std::vector<float>,
     @param[in] _eulerTransformation  the rotation order applied on the euler angle
    @return the reef_mgs euler angle instancy created
    */
    template<class T>
    reef_msgs::RodriguezParameter fromAnyTypeToRodriguezParameter(const T &_msgs ){
        double x,y,z;
        if constexpr(std::is_same<geometry_msgs::Vector3 , T>::value){
            x = _msgs.x;
            y = _msgs.y;
            z = _msgs.z;
        }else if constexpr(std::is_same<geometry_msgs::Vector3Stamped , T>::value) {
            x = _msgs.vector.x;
            y = _msgs.vector.y;
            z = _msgs.vector.z;
        }else if constexpr((std::is_same<Eigen::Matrix<double,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3d , T>::value)
                           || (std::is_same<Eigen::Matrix<float,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3f, T>::value) ) {
            x = _msgs.coeff(0,0);
            y = _msgs.coeff(1,0);
            z = _msgs.coeff(2,0);
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            x = _msgs.at(0);
            y = _msgs.at(1);
            z = _msgs.at(2);
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::RodriguezParameter(x,y,z);
    }
    /*!
    @brief Generate a reef_msgs rotation matrix from differents type of messages
    @param[in] msgs the msgs to be turn into a reef_msgs rotation matrix. Right now, the type supported are:
     Eigen::Matrix3d,
     Eigen::Matrix3f,
     Eigen::Matrix<double,3,3>,
     Eigen::Matrix<float,3, 3>,
    @return the reef_mgs rotation matrix instancy created
    */
    template<class T>
    reef_msgs::RotationMatrix fromAnyTypeToRotationMatrix(const T &_msgs ){
        Eigen::Matrix3d mat;
        if constexpr((std::is_same<Eigen::Matrix3d, T>::value)
                     || (std::is_same<Eigen::Matrix<double,3,3>, T>::value)
                     || (std::is_same<Eigen::Matrix3f, T>::value)
                     || (std::is_same<Eigen::Matrix<float,3,3>, T>::value)){
            mat << _msgs;
        } else{
            throw "This type isn't defined";
        }
        return reef_msgs::RotationMatrix(mat);
    }
    /*!
    @brief transform a reef_msgs quaternion to another possible quaternion c++ type
    @param[in] _Quaternion the quaternion to be transformed
    @return the c++ type returned . Right now, the type supported are:
    geometry_msgs::Quaternion,
    geometry_msgs::QuaternionStamped,
    Eigen::Quaterniond,
    Eigen::Matrix<double,4, 1>,
    Eigen::Vector4d ,
    Eigen::Matrix<float,4, 1>,
    Eigen::Vector4f,
    std::vector<double>,
    std::vector<float>,
    tf2::Quaternion,
    geometry_msgs::Transform,
    geometry_msgs::TransformStamped,
    */
    template<class T>
    T fromQuaternionToAnyType(const reef_msgs::Quaternion &_Quaternion){
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

    /*!
    @brief transform a reef_msgs axiangle to another possible axis angle c++ type
    @param[in] _axisAngle the axisangle to be transformed
    @return the c++ type returned . Right now, the type supported are:
    Eigen::Matrix<double,4, 1>,
    Eigen::Vector4d ,
    Eigen::Matrix<float,4, 1>,
    Eigen::Vector4f,
    std::vector<double>,
    std::vector<float>,
    */
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
    /*!
    @brief transform a reef_msgs DCM to another possible DCM c++ type
    @param[in] _DCM the DCM to be transformed
    @return the c++ type returned . Right now, the type supported are:
    Eigen::Matrix<double,3, 3>,
    Eigen::Matrix3d ,
    Eigen::Matrix<float,3, 3>,
    Eigen::Matrix3f,
    */
    template<class T>
    T fromDCMToAnyType(const reef_msgs::DCM &_DCM){
        if constexpr((std::is_same<Eigen::Matrix3d, T>::value)
                     || (std::is_same<Eigen::Matrix<double,3,3>, T>::value)
                     ){
            Eigen::Matrix3d mat;
            mat << _DCM.m_DCM;
            return mat;
        }else if constexpr((std::is_same<Eigen::Matrix3f, T>::value)
                              || (std::is_same<Eigen::Matrix<float,3,3>, T>::value)) {
            Eigen::Matrix3f mat;
            mat << _DCM.m_DCM;
            return mat;
        } else{
            throw "This type isn't defined";
        }
    }
    /*!
    @brief transform a reef_msgs euler angle to another possible euler angle c++ type
    @param[in] _eulerAngle the eulerangle to be transformed
    @return the c++ type returned . Right now, the type supported are:
     geometry_msgs::Vector3,
    geometry_msgs::Vector3Stamped,
    Eigen::Matrix<double,3, 1>,
    Eigen::Vector3d ,
    Eigen::Matrix<float,3, 1>,
    Eigen::Vector3f,
    std::vector<double>,
    std::vector<float>,
    */
    template<class T>
    T fromEulerAngleToAnyType(const reef_msgs::EulerAngle &_eulerAngle){
        if constexpr(std::is_same<geometry_msgs::Vector3, T>::value){
            geometry_msgs::Vector3 outputAngle;
            outputAngle.x = _eulerAngle.yaw();
            outputAngle.y = _eulerAngle.pitch();
            outputAngle.z = _eulerAngle.roll();
            return outputAngle;
        }else if constexpr(std::is_same<geometry_msgs::Vector3Stamped , T>::value) {
            geometry_msgs::Vector3Stamped outputAngle;
            outputAngle.vector.x = _eulerAngle.yaw();
            outputAngle.vector.y = _eulerAngle.pitch();
            outputAngle.vector.z = _eulerAngle.roll();
            return outputAngle;
        }else if constexpr((std::is_same<Eigen::Matrix<double,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3d , T>::value)){
            Eigen::Matrix<double,3, 1> outputAngle;
            outputAngle << _eulerAngle.getEulerAngle();
            return outputAngle;
        }else if constexpr((std::is_same<Eigen::Matrix<float,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3f, T>::value) ) {
            Eigen::Matrix<float,3, 1> outputAngle;
            outputAngle << _eulerAngle.getEulerAngle();
            return outputAngle;
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            std::vector<double> outputAngle;
            outputAngle.push_back(_eulerAngle.yaw());
            outputAngle.push_back(_eulerAngle.pitch());
            outputAngle.push_back(_eulerAngle.roll());
            return outputAngle;
        } else{
            throw "This type isn't defined";
        }
    }
    /*!
    @brief transform a reef_msgs Rodriguez parameter to another possible rodriguez parameter c++ type
    @param[in] _rodriguezParameter the rodriguez parameter to be transformed
    @return the c++ type returned.  Right now, the type supported are:
     geometry_msgs::Vector3,
     geometry_msgs::Vector3Stamped,
     Eigen::Matrix<double,3, 1>,
     Eigen::Matrix<float,3, 1>,
     Eigen::Vector3d,
     Eigen::Vector3f,
     std::vector<double>,
     std::vector<float>,
    */
    template<class T>
    T fromRodriguezParameterToAnyType(const reef_msgs::RodriguezParameter &_rodriguezParameter){
        if constexpr(std::is_same<geometry_msgs::Vector3, T>::value){
            geometry_msgs::Vector3 outputAngle;
            outputAngle.x = _rodriguezParameter.x();
            outputAngle.y = _rodriguezParameter.y();
            outputAngle.z = _rodriguezParameter.z();
            return outputAngle;
        }else if constexpr(std::is_same<geometry_msgs::Vector3Stamped , T>::value) {
            geometry_msgs::Vector3Stamped outputAngle;
            outputAngle.vector.x = _rodriguezParameter.x();
            outputAngle.vector.y = _rodriguezParameter.y();
            outputAngle.vector.z = _rodriguezParameter.z();
            return outputAngle;
        }else if constexpr((std::is_same<Eigen::Matrix<double,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3d , T>::value)){
            Eigen::Matrix<double,3, 1> outputAngle;
            outputAngle << _rodriguezParameter.getParameter();
            return outputAngle;
        }else if constexpr((std::is_same<Eigen::Matrix<float,3, 1>, T>::value)
                           || (std::is_same<Eigen::Vector3f, T>::value) ) {
            Eigen::Matrix<float,3, 1> outputAngle;
            outputAngle << _rodriguezParameter.getParameter();
            return outputAngle;
        }else if constexpr((std::is_same<std::vector<double>, T>::value)
                           || (std::is_same<std::vector<float>, T>::value)) {
            std::vector<double> outputAngle;
            outputAngle.push_back(_rodriguezParameter.x());
            outputAngle.push_back(_rodriguezParameter.y());
            outputAngle.push_back(_rodriguezParameter.z());
            return outputAngle;
        } else{
            throw "This type isn't defined";
        }
    }
    /*!
    @brief transform a reef_msgs Rotation matrix to another possible rotation matrix c++ type
    @param[in] _rotationMatrix the rotation matrix to be transformed
    @return the c++ type returned . Right now, the type supported are:
    Eigen::Matrix<double,3, 3>,
    Eigen::Matrix3d ,
    Eigen::Matrix<float,3, 3>,
    Eigen::Matrix3f,
    */
    template<class T>
    T fromRotationMatrixToAnyType(const reef_msgs::RotationMatrix &_rotationMatrix){
        if constexpr((std::is_same<Eigen::Matrix3d, T>::value)
                     || (std::is_same<Eigen::Matrix<double,3,3>, T>::value)
                ){
            Eigen::Matrix3d mat;
            mat << _rotationMatrix.m_RotationMatrix;
            return mat;
        }else if constexpr((std::is_same<Eigen::Matrix3f, T>::value)
                           || (std::is_same<Eigen::Matrix<float,3,3>, T>::value)) {
            Eigen::Matrix3f mat;
            mat << _rotationMatrix.m_RotationMatrix;
            return mat;
        } else{
            throw "This type isn't defined";
        }
    }
}


