//
// Created bb paul on 10/5/20.
//

#ifndef REEF_MSGS_RODRIGUEZPARAMETER_H
#define REEF_MSGS_RODRIGUEZPARAMETER_H
#pragma once
#include "AngleRepresentationInterface.h"

namespace reef_msgs{
    class RodriguezParameter: public AngleRepresentationInterface{

    public:
        RodriguezParameter(const double &_a, const double &_b, const double &_c, const double &_d);
        RodriguezParameter(const Eigen::Matrix<double, 4, 1> &_parameters);
        RodriguezParameter(const RodriguezParameter &other);
        void setParameter(const double &_a, const double &_b, const double &_c, const double &_d);
        auto getParameter() const -> const Eigen::Matrix<double,4,1>&;

        auto a() const ->  double ;
        void setA(const double &_a);
        auto b() const -> double;
        void setB(const double &_b);
        auto c() const-> double;
        void setC(const double &_c);
        auto d() const -> double;
        void setD(const double &_d);

        RodriguezParameter& operator=(const RodriguezParameter & _other);
        friend std::ostream& operator<<(std::ostream& os,const RodriguezParameter & _inst);
        ~RodriguezParameter() {};
    private:
        Eigen::Matrix<double,4,1> m_parameter; //!< this matrix store the parameters value in the "abcd" order

    };

};



#endif //REEF_MSGS_RODRIGUEZPARAMETER_H
