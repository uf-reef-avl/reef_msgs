//
// Created by paul on 10/19/20.
//
#pragma once
#ifndef SRC_REEFMSGSCONVERSIONAPI_H
#define SRC_REEFMSGSCONVERSIONAPI_H
#include "ReefObjectConversion.h"


namespace reef_msgs{
    /*!
    @brief transform a S c++ type quaternion representation to a T type axisangle representation
    @param[in] _inputMsgs the S type inputMsgs as a quaternion representation
    @return the c++ T type returned as a axisangle representation
    */
    template<class S, class T>
    T fromQuaternionToAxisAngle(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    /*!
    @brief transform a S c++ type quaternion representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a quaternion representation
    @return the c++ T type returned as a DCM representation
    */
    template<class S, class T>
    T fromQuaternionToDCM(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    /*!
    @brief transform a S c++ type quaternion representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a quaternion representation
    @param[in] _eulerTransformation the rotation order applied to the original  euler angle representation
    @return the c++ T type returned as a DCM representation
*/
    template<class S, class T>
    T fromQuaternionToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    /*!
    @brief transform a S c++ type quaternion representation to a T type Rodrigue parameter representation
    @param[in] _inputMsgs the S type inputMsgs as a quaternion representation
    @return the c++ T type returned as a rodriguez parameter representation
    */
    template<class S, class T>
    T fromQuaternionToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    /*!
    @brief transform a S c++ type quaternion representation to a T type rotation matrix representation
    @param[in] _inputMsgs the S type inputMsgs as a quaternion representation
    @return the c++ T type returned as a rotation matrix representation
    */
    template<class S, class T>
    T fromQuaternionToRotationMatrix(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };
    /*!
    @brief transform a S c++ type axis angle representation to a T type quaternion representation
    @param[in] _inputMsgs the S type inputMsgs as a axis angle representation
    @return the c++ T type returned as a quaternion representation
    */
    template<class S, class T>
    T fromAxisAngleToQuaternion(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));

    };
    /*!
    @brief transform a S c++ type axis angle representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a axis angle representation
    @return the c++ T type returned as a DCM representation
    */
    template<class S, class T>
    T fromAxisAngleToDCM(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    /*!
    @brief transform a S c++ type axis angle representation to a T type euler angle representation
    @param[in] _inputMsgs the S type inputMsgs as a axis angle representation
    @param[in] _eulerTransformation the rotation order applied to the original  euler angle representation
    @return the c++ T type returned as a euler angle representation
    */
    template<class S, class T>
    T fromAxisAngleToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    /*!
    @brief transform a S c++ type axis angle representation to a T type rodriguez parameter representation
    @param[in] _inputMsgs the S type inputMsgs as a axis angle representation
    @return the c++ T type returned as a rodriguez parameter representation
    */
    template<class S, class T>
    T fromAxisAngleToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    /*!
    @brief transform a S c++ type axis angle representation to a T type rotation matrix representation
    @param[in] _inputMsgs the S type inputMsgs as a axis angle representation
    @return the c++ T type returned as a rotation matrix representation
    */
    template<class S, class T>
    T fromAxisAngleToRotationMatrix(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };
    /*!
    @brief transform a S c++ type DCM representation to a T type axis angle representation
    @param[in] _inputMsgs the S type inputMsgs as a DCM representation
    @return the c++ T type returned as a axis angle representation
    */
    template<class S, class T>
    T fromDCMToAxisAngle(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    /*!
    @brief transform a S c++ type DCM representation to a T type quaternion representation
    @param[in] _inputMsgs the S type inputMsgs as a DCM representation
    @return the c++ T type returned as a quaternion representation
    */
    template<class S, class T>
    T fromDCMToQuaternion(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    };
    /*!
    @brief transform a S c++ type DCM representation to a T type euler angle representation
    @param[in] _inputMsgs the S type inputMsgs as a DCM representation
    @param[in] _eulerTransformation the rotation order applied to the original  euler angle representation
    @return the c++ T type returned as a euler angle representation
    */
    template<class S, class T>
    T fromDCMToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    /*!
    @brief transform a S c++ type DCM representation to a T type rodriguez parameter representation
    @param[in] _inputMsgs the S type inputMsgs as a DCM representation
    @return the c++ T type returned as a rodriguez parameter representation
    */
    template<class S, class T>
    T fromDCMToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    /*!
    @brief transform a S c++ type DCM representation to a T type rotation matrix representation
    @param[in] _inputMsgs the S type inputMsgs as a DCM representation
    @return the c++ T type returned as a rotation matrix representation
    */
    template<class S, class T>
    T fromDCMToRotationMatrix(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };
    /*!
    @brief transform a S c++ type euler angle representation to a T type axis angle representation
    @param[in] _inputMsgs the S type inputMsgs as a euler angle representation
    @param[in] _inputEulerTransformation the rotation order applied to the euler angle
    @return the c++ T type returned as a axis angle representation
    */
    template<class S, class T>
    T fromEulerAngleToAxisAngle(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    /*!
    @brief transform a S c++ type euler angle representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a euler angle representation
    @param[in] _inputEulerTransformation the rotation order applied to the euler angle
    @return the c++ T type returned as a DCM representation
    */
    template<class S, class T>
    T fromEulerAngleToDCM(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    /*!
    @brief transform a S c++ type euler angle representation to a T type quaternion representation
    @param[in] _inputMsgs the S type inputMsgs as a euler angle representation
    @param[in] _inputEulerTransformation the rotation order applied to the euler angle
    @return the c++ T type returned as a quaternion representation
    */
    template<class S, class T>
    T fromEulerAngleToQuaternion(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    };
    /*!
    @brief transform a S c++ type euler angle representation to a T type rodriguez parameter representation
    @param[in] _inputMsgs the S type inputMsgs as a euler angle representation
    @param[in] _inputEulerTransformation the rotation order applied to the euler angle
    @return the c++ T type returned as a rodriguez parameter representation
    */
    template<class S, class T>
    T fromEulerAngleToRodriguezParameter(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    /*!
    @brief transform a S c++ type euler angle representation to a T type rotation matrix representation
    @param[in] _inputMsgs the S type inputMsgs as a euler angle representation
    @param[in] _inputEulerTransformation the rotation order applied to the euler angle
    @return the c++ T type returned as a rotation matrix representation
    */
    template<class S, class T>
    T fromEulerAngleToRotationMatrix(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321") {
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };
    /*!
    @brief transform a S c++ type rodriguez parameter representation to a T type axis angle representation
    @param[in] _inputMsgs the S type inputMsgs as a rodriguez parameter representation
    @return the c++ T type returned as a axis angle representation
    */
    template<class S, class T>
    T fromRodriguezParameterToAxisAngle(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    /*!
    @brief transform a S c++ type rodriguez parameter representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a rodriguez parameter representation
    @return the c++ T type returned as a DCM representation
    */
    template<class S, class T>
    T fromRodriguezParameterToDCM(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    /*!
    @brief transform a S c++ type rodriguez parameter representation to a T type euler angle representation
    @param[in] _inputMsgs the S type inputMsgs as a rodriguez parameter representation
    @param[in] _eulerTransformation the rotation order applied to the original  euler angle representation
    @return the c++ T type returned as a euler angle representation
    */
    template<class S, class T>
    T fromRodriguezParameterToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    /*!
    @brief transform a S c++ type rodriguez parameter representation to a T type quaternion representation
    @param[in] _inputMsgs the S type inputMsgs as a rodriguez parameter representation
    @return the c++ T type returned as a quaternion representation
    */
    template<class S, class T>
    T fromRodriguezParameterToQuaternion(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    }
    /*!
    @brief transform a S c++ type rodriguez parameter representation to a T type rotation matrix representation
    @param[in] _inputMsgs the S type inputMsgs as a rodriguez parameter representation
    @return the c++ T type returned as a rotation matrix representation
    */
    template<class S, class T>
    T fromRodriguezParameterToRotationMatrix(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    }
    /*!
    @brief transform a S c++ type rotation matrix representation to a T type axis angle representation
    @param[in] _inputMsgs the S type inputMsgs as a rotation matrix representation
    @return the c++ T type returned as a axis angle representation
    */
    template<class S, class T>
    T fromRotationMatrixToAxisAngle(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    }
    /*!
    @brief transform a S c++ type rotation matrix representation to a T type DCM representation
    @param[in] _inputMsgs the S type inputMsgs as a rotation matrix representation
    @return the c++ T type returned as a DCM representation
    */
    template<class S, class T>
    T fromRotationMatrixToDCM(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    }
    /*!
    @brief transform a S c++ type rotation matrix representation to a T type euler angle representation
    @param[in] _inputMsgs the S type inputMsgs as a rotation matrix representation
    @param[in] _eulerTransformation the rotation order applied to the original  euler angle representation
    @return the c++ T type returned as a euler angle representation
    */
    template<class S, class T>
    T fromRotationMatrixToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation ="321"){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));

    }
    /*!
    @brief transform a S c++ type rotation matrix representation to a T type rodriguez parameter representation
    @param[in] _inputMsgs the S type inputMsgs as a rotation matrix representation
    @return the c++ T type returned as a rodriguez parameter representation
    */
    template<class S, class T>
    T fromRotationMatrixToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));

    }
    /*!
    @brief transform a S c++ type rotation matrix representation to a T type quaternion representation
    @param[in] _inputMsgs the S type inputMsgs as a rotation matrix representation
    @return the c++ T type returned as a quaternion representation
    */
    template<class S, class T>
    T fromRotationMatrixToQuaternion(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));

    }
}


#endif //SRC_REEFMSGSCONVERSIONAPI_H
