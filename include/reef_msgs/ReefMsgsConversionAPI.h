//
// Created by paul on 10/19/20.
//
#pragma once
#ifndef SRC_REEFMSGSCONVERSIONAPI_H
#define SRC_REEFMSGSCONVERSIONAPI_H
#include "ReefObjectConversion.h"


namespace reef_msgs{
    template<class S, class T>
    T fromQuaternionToAxisAngle(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    template<class S, class T>
    T fromQuaternionToDCM(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    template<class S, class T>
    T fromQuaternionToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    template<class S, class T>
    T fromQuaternionToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    template<class S, class T>
    T fromQuaternionToRotationMatrix(const S &_inputMsgs){
        reef_msgs::Quaternion inputAngle = reef_msgs::fromAnyTypeToQuaternion<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };

    template<class S, class T>
    T fromAxisAngleToQuaternion(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));

    };
    template<class S, class T>
    T fromAxisAngleToDCM(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    template<class S, class T>
    T fromAxisAngleToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    template<class S, class T>
    T fromAxisAngleToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    template<class S, class T>
    T fromAxisAngleToRotationMatrix(const S &_inputMsgs){
        reef_msgs::AxisAngle inputAngle = reef_msgs::fromAnyTypeToAxisAngle<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };

    template<class S, class T>
    T fromDCMToAxisAngle(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    template<class S, class T>
    T fromDCMToQuaternion(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    };
    template<class S, class T>
    T fromDCMToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    template<class S, class T>
    T fromDCMToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    template<class S, class T>
    T fromDCMToRotationMatrix(const S &_inputMsgs){
        reef_msgs::DCM inputAngle = reef_msgs::fromAnyTypeToDCM<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };

    template<class S, class T>
    T fromEulerAngleToAxisAngle(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    template<class S, class T>
    T fromEulerAngleToDCM(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    template<class S, class T>
    T fromEulerAngleToQuaternion(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    };
    template<class S, class T>
    T fromEulerAngleToRodriguezParameter(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321"){
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));
    };
    template<class S, class T>
    T fromEulerAngleToRotationMatrix(const S &_inputMsgs, const std::string &_inputEulerTransformation = "321") {
        reef_msgs::EulerAngle inputAngle = reef_msgs::fromAnyTypeToEulerAngle<S>(_inputMsgs, _inputEulerTransformation);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    };

    template<class S, class T>
    T fromRodriguezParameterToAxisAngle(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    };
    template<class S, class T>
    T fromRodriguezParameterToDCM(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    };
    template<class S, class T>
    T fromRodriguezParameterToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation = "321"){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));
    };
    template<class S, class T>
    T fromRodriguezParameterToQuaternion(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));
    }
    template<class S, class T>
    T fromRodriguezParameterToRotationMatrix(const S &_inputMsgs){
        reef_msgs::RodriguezParameter inputAngle = reef_msgs::fromAnyTypeToRodriguezParameter<S>(_inputMsgs);
        return  reef_msgs::fromRotationMatrixToAnyType<T>(reef_msgs::RotationMatrix(inputAngle.toRotationMatrix()));
    }

    template<class S, class T>
    T fromRotationMatrixToAxisAngle(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromAxisAngleToAnyType<T>(reef_msgs::AxisAngle(inputAngle.toAxisAngle()));
    }
    template<class S, class T>
    T fromRotationMatrixToDCM(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromDCMToAnyType<T>(reef_msgs::DCM(inputAngle.toDCM()));
    }
    template<class S, class T>
    T fromRotationMatrixToEulerAngle(const S &_inputMsgs, const std::string &_eulerTransformation ="321"){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromEulerAngleToAnyType<T>(reef_msgs::EulerAngle(inputAngle.toEulerAngle(_eulerTransformation),_eulerTransformation));

    }
    template<class S, class T>
    T fromRotationMatrixToRodriguezParameter(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromRodriguezParameterToAnyType<T>(reef_msgs::RodriguezParameter(inputAngle.toRodriguezParameter()));

    }
    template<class S, class T>
    T fromRotationMatrixToQuaternion(const S &_inputMsgs){
        reef_msgs::RotationMatrix inputAngle = reef_msgs::fromAnyTypeToRotationMatrix<S>(_inputMsgs);
        return  reef_msgs::fromQuaternionToAnyType<T>(reef_msgs::Quaternion(inputAngle.toQuaternion()));

    }
}


#endif //SRC_REEFMSGSCONVERSIONAPI_H
