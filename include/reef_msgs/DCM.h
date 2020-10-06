//
// Created by paul on 10/5/20.
//

#ifndef REEF_MSGS_DCM_H
#define REEF_MSGS_DCM_H

#include "AngleRepresentationInterface.h"

namespace reef_msgs {
    class DCM : public reef_msgs::AngleRepresentationInterface {
    public:
        DCM(const Eigen::Matrix<double, 3, 3> &_DCM);

        DCM( DCM &other);

        void setDCM(const Eigen::Matrix<double, 3, 3> &_DCM);

        //to avoid to redefine all the function from eigen, we let the user access the matrix directly to change it;
        auto getDCM() -> Eigen::Matrix<double, 3, 3> &;

        DCM &operator=( DCM &_other);

        friend std::ostream &operator<<(std::ostream &os, const DCM &_inst);

        ~DCM() {};

    private:
        Eigen::Matrix<double, 3, 3> m_DCM; //!< this matrix store all the informations of the direction cosine matrix;

    };
}


#endif //REEF_MSGS_DCM_H
