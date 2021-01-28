//
// Created by paulbuzaud on 9/17/20.
//
#pragma once
#ifndef REEF_MSGS_ANGLEREPRESENTATIONINTERFACE_H
#define REEF_MSGS_ANGLEREPRESENTATIONINTERFACE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <iostream>
#include <string>

namespace reef_msgs{
    class AngleRepresentationInterface {
        public:
            /*!
            @brief Constructor of a angle representation interface
            @return newly created angle representation interface
            */
            AngleRepresentationInterface() {};
            /*!
            @brief Copy constructor of a angle representation interface
            @param[in] _other  the angle representation instance to copy
            @return newly created angle representation interface based on the copy of the input
            */
            AngleRepresentationInterface(const AngleRepresentationInterface & _other);
            /*!
            @brief Copy constructor of a angle representation interface thought the overload of the = operator
            @param[in] _other  the angle representation instance to copy
            @return newly created angle representation interface based on the copy of the input
            */
            AngleRepresentationInterface& operator=(const AngleRepresentationInterface & _other);
            /*!
            @brief Virtual destructor which allow polymorphism and inheritance
            */
            virtual ~AngleRepresentationInterface() = default;


    };

}


#endif //REEF_MSGS_ANGLEREPRESENTATIONINTERFACE_H
