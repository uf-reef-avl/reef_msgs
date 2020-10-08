//
// Created bb paul on 10/5/20.
//
#pragma once
#ifndef REEF_MSGS_RODRIGUEZPARAMETER_H
#define REEF_MSGS_RODRIGUEZPARAMETER_H
#include "AngleRepresentationInterface.h"

namespace reef_msgs{
    class RodriguezParameter: public AngleRepresentationInterface{

    public:
        RodriguezParameter(const double &_x, const double &_y, const double &_z);
        RodriguezParameter(const Eigen::Matrix<double, 3, 1> &_parameters);
        RodriguezParameter(const RodriguezParameter &other);
        void setParameter(const double &_x, const double &_y, const double &_z);
        auto getParameter() const -> const Eigen::Matrix<double,3,1>&;

        auto x() const ->  double ;
        void setX(const double &_x);
        auto y() const -> double;
        void setY(const double &_y);
        auto z() const-> double;
        void setZ(const double &_z);


        RodriguezParameter& operator=(const RodriguezParameter & _other);
        friend std::ostream& operator<<(std::ostream& os,const RodriguezParameter & _inst);
        ~RodriguezParameter() {};
    private:
        Eigen::Matrix<double,3,1> m_parameter; //!< this matrix store the parameters value in the "xyz" order

    };

};



#endif //REEF_MSGS_RODRIGUEZPARAMETER_H
