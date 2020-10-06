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

namespace reef_msgs{
    class AngleRepresentationInterface {
        public:
            AngleRepresentationInterface() {};
            AngleRepresentationInterface(const AngleRepresentationInterface & _other);
            AngleRepresentationInterface& operator=(const AngleRepresentationInterface & _other);
            virtual ~AngleRepresentationInterface() = default;


    };

}


#endif //REEF_MSGS_ANGLEREPRESENTATIONINTERFACE_H
