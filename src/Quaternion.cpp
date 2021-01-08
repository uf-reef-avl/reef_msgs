//
// Created by paulbuzaud on 9/17/20.
//

#include "reef_msgs/Quaternion.h"


namespace reef_msgs {
    auto Quaternion::eye() -> Quaternion {
        Quaternion instEye(0, 0, 0, 1);
        return instEye;
    }

    auto Quaternion::rand() -> Quaternion {
        Quaternion instRand(((double) std::rand() / RAND_MAX), ((double) std::rand() / RAND_MAX),
                            ((double) std::rand() / RAND_MAX), ((double) std::rand() / RAND_MAX));
        instRand.normalize();
        return instRand;
    }

    auto Quaternion::quaternionMultiplication(Eigen::Matrix<double, 4, 1> p,
                                              Eigen::Matrix<double, 4, 1> q) -> Eigen::Matrix<double, 4, 1> {
        double pw = p(3, 0);

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d p_skew;
        p_skew = skew(p.head(3));

        Eigen::Matrix4d A;
        A.block<3, 3>(0, 0) = pw * I * p_skew;
        A.block<4, 1>(0, 3) = p;
        A.block<1, 4>(3, 0) = -p.transpose();
        A(3, 3) = pw;
        return A * q;
    }

    auto Quaternion::quatMult(Eigen::Quaterniond q, Eigen::Quaterniond p) -> Eigen::Quaterniond {

        Eigen::Vector4d p_eigen;
        p_eigen = p.coeffs();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d q_skew;
        q_skew = skew(q.vec());

        Eigen::Matrix4d A;
        A.block<3, 3>(0, 0) = q.w() * I - q_skew;
        A.block<3, 1>(0, 3) = q.vec();
        A.block<1, 3>(3, 0) = -q.vec();
        A(3, 3) = q.w();
        Eigen::Vector4d qp;
        qp = A * p_eigen;
        Eigen::Quaterniond q_times_p;
        q_times_p.vec() << qp(0), qp(1), qp(2);
        q_times_p.w() = qp(3);
        return q_times_p;
    }

    auto Quaternion::quaternionMultiplication(Eigen::Quaterniond p, Eigen::Quaterniond q) -> Eigen::Quaterniond {
        Eigen::Matrix<double, 4, 1> p_temp;
        Eigen::Matrix<double, 4, 1> q_temp;
        Eigen::Matrix<double, 4, 1> multiplicant;

        p_temp = quat2vector(p);
        q_temp = quat2vector(q);

        multiplicant = quaternionMultiplication(p_temp, q_temp);

        return vector2quat(multiplicant);
    }

    auto Quaternion::quaternionToRotation(Eigen::Quaterniond q) -> Eigen::Matrix3d {
        Eigen::Matrix3d I;
        I.setIdentity();

        Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q.x(), q.y(), q.z()));
        Eigen::Matrix3d rotation_matrix = I - 2 * q.w() * skew_mat + 2 * skew_mat * skew_mat;

        return rotation_matrix;
    }

    auto Quaternion::quaternionToRotation(geometry_msgs::Quaternion q) -> Eigen::Matrix3d {
        Eigen::Quaterniond q_temp;
        q_temp.x() = q.x;
        q_temp.y() = q.y;
        q_temp.z() = q.z;
        q_temp.w() = q.w;

        return quaternionToRotation(q_temp);
    }

    auto Quaternion::toRotation() -> Eigen::Matrix3d {
        Eigen::Quaterniond q_temp;
        q_temp.x() = m_q.x();
        q_temp.y() = m_q.y();
        q_temp.z() = m_q.z();
        q_temp.w() = m_q.w();

        return quaternionToRotation(q_temp);
    };

    auto Quaternion::toDCM() -> Eigen::Matrix3d {
        Eigen::Matrix3d m = this->Xi().transpose() * this->Psi();
        return m;
    };

    auto Quaternion::toRotationMatrix() -> Eigen::Matrix3d {
        Eigen::Matrix3d m = this->toDCM().transpose();
        return m;
    }

    auto Quaternion::toAxisAngle() -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 1> axisAngle;
        double p = 2 * acos(this->w());
        double sp = sin(p / 2);
        double angle = this->x() / sp * p;
        double x = this->y() / sp * p;
        double y = this->z() / sp * p;
        double z = sqrt(1 - x*x - y*y);
        axisAngle << x, y, z, angle;
        return axisAngle;
    }

    auto Quaternion::toRodriguezParameter() -> Eigen::Matrix<double, 3, 1> {
        Eigen::Matrix<double, 3, 1> rodriguezParameter;
        double x = this->x() / this->w();
        double y = this->y() / this->w();
        double z = this->z() / this->w();
        rodriguezParameter << x, y, z;
        return rodriguezParameter;
    }

    auto Quaternion::toEulerAngle(const std::string &_eulerTransformation) -> Eigen::Matrix<double, 3, 1> {
        Eigen::Matrix<double, 3, 1> eulerAngle;
        if (_eulerTransformation == "121") {
            double t1 = atan2(this->z(), this->y());
            double t2 = atan2(this->x(), this->w());
            double e1 = t1 + t2;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->x() * this->x()));
            double e3 = t2 - t1;
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "123") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(-2 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            double e2 = asin(2 * (q1 * q3 + q0 * q2));
            double e3 = atan2(-2 * (q1 * q2 - q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "131") {
            double t1 = atan2(this->y(), this->z());
            double t2 = atan2(this->x(), this->w());
            double e1 = t2 - t1;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->x() * this->x()));
            double e3 = t2 + t1;
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "132") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            double e2 = asin(-2 * (q1 * q2 - q0 * q3));
            double e3 = atan2(2 * (q1 * q3 + q0 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "212") {
            double t1 = atan2(this->z(), this->x());
            double t2 = atan2(this->y(), this->w());
            double e1 = t2 - t1;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->y() * this->y()));
            double e3 = t2 + t1;
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "213") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(2 * (q1 * q3 + q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            double e2 = asin(-2 * (q2 * q3 - q0 * q1));
            double e3 = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "231") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(-2 * (q1 * q3 - q0 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            double e2 = asin(2 * (q1 * q2 + q0 * q3));
            double e3 = atan2(-2 * (q2 * q3 - q0 * q1), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "232") {
            double t1 = atan2(this->x(), this->z());
            double t2 = atan2(this->y(), this->w());
            double e1 = t1 + t2;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->y() * this->y()));
            double e3 = t2 - t1;
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "312") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(-2 * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            double e2 = asin(2 * (q2 * q3 + q0 * q1));
            double e3 = atan2(-2 * (q1 * q3 - q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "313") {
            double t1 = atan2(this->y(), this->x());
            double t2 = atan2(this->z(), this->w());
            double e1 = t1 + t2;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->z() * this->z()));
            double e3 = t2 - t1;
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "321") {
            double q0 = this->w();
            double q1 = this->x();
            double q2 = this->y();
            double q3 = this->z();
            double e1 = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            double e2 = asin(-2 * (q1 * q3 - q0 * q2));
            double e3 = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            eulerAngle << e1, e2, e3;
        } else if (_eulerTransformation == "323") {
            double t1 = atan2(this->x(), this->y());
            double t2 = atan2(this->z(), this->w());
            double e1 = t2 - t1;
            double e2 = 2 * acos(sqrt(this->w() * this->w() + this->z() * this->z()));
            double e3 = t2 + t1;
            eulerAngle << e1, e2, e3;
        } else {
            throw std::invalid_argument("this euler transformation isn't known");
        }
        return eulerAngle;
    }

    auto Quaternion::fromDCMtoQuaternion(const Eigen::Matrix<double, 3, 3> &_m) -> Eigen::Matrix<double, 4, 1> {
        auto gamma = _m.trace();
        double w2 = (1 + gamma) / 4;
        Eigen::Matrix<double, 3, 1> Ckk = _m.diagonal();
        Eigen::Matrix<double, 3, 1> quat2temp =
                (Eigen::Matrix<double, 3, 1>::Ones() + 2 * Ckk - (gamma * Eigen::Matrix<double, 3, 1>::Ones())) / 4.;
        Eigen::Matrix<double, 4, 1> quat2;
        quat2 << quat2temp(0, 0), quat2temp(1, 0), quat2temp(2, 0), w2;
        int maxIndex = 0;
        double max = quat2[0];
        for (int i = 0; i < 4; i++) {
            if (quat2(i, 0) > max) {
                max = quat2(i, 0);
                maxIndex = i;
            }
        }
        Eigen::Matrix<double, 4, 1> q;
        q.setZero();
        q(maxIndex, 0) = sqrt(quat2(maxIndex, 0));
        double d = 4. * q[maxIndex];
        if (maxIndex == 3) {
            q(0, 0) = (_m(1, 2) - _m(2, 1)) / d;
            q(1, 0) = (_m(2, 0) - _m(0, 2)) / d;
            q(2, 0) = (_m(0, 1) - _m(1, 0)) / d;
        } else if (maxIndex == 0) {
            q(3, 0) = (_m(1, 2) - _m(2, 1)) / d;
            q(1, 0) = (_m(0, 1) + _m(1, 0)) / d;
            q(2, 0) = (_m(2, 0) + _m(0, 2)) / d;
        } else if (maxIndex == 1) {
            q(3, 0) = (_m(2, 0) - _m(0, 2)) / d;
            q(0, 0) = (_m(0, 1) + _m(1, 0)) / d;
            q(2, 0) = (_m(1, 2) + _m(2, 1)) / d;
        } else if (maxIndex == 2) {
            q(3, 0) = (_m(0, 1) - _m(1, 0)) / d;
            q(0, 0) = (_m(2, 0) + _m(0, 2)) / d;
            q(1, 0) = (_m(1, 2) + _m(2, 1)) / d;
        }
        q.normalize();

        return q;
    }

    auto Quaternion::fromAngleAxistoQuaternion(const double &_angle,
                                               const Eigen::Matrix<double, 3, 1> &_axis) -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 1> quat;
        quat.setZero();
        Eigen::Matrix<double, 3, 1> q = sin(_angle / 2.) * _axis.normalized();
        quat.block<3, 1>(0, 0) << q;
        quat(3, 0) = cos(_angle / 2.);
        return quat;
    }

    double Quaternion::norm() {
        return m_q.norm();
    }

    void Quaternion::normalize() {
        m_q.normalize();
    }

    void Quaternion::setQuaternion(const double &_x, const double &_y, const double &_z, const double &_w) {
        m_q(0, 0) = _x;
        m_q(1, 0) = _y;
        m_q(2, 0) = _z;
        m_q(3, 0) = _w;
    }

    auto Quaternion::getQuaternion() const -> const Eigen::Matrix<double, 4, 1> & {
        return m_q;
    }

    auto Quaternion::x() const -> double {
        return m_q(0, 0);
    }

    void Quaternion::setX(const double &_x) {
        m_q(0, 0) = _x;
    }

    auto Quaternion::y() const -> double {
        return m_q(1, 0);
    }

    void Quaternion::setY(const double &_y) {
        m_q(1, 0) = _y;
    }

    auto Quaternion::z() const -> double {
        return m_q(2, 0);
    }

    void Quaternion::setZ(const double &_z) {
        m_q(2, 0) = _z;
    }

    auto Quaternion::w() const -> double {
        return m_q(3, 0);
    }

    void Quaternion::setW(const double &_w) {
        m_q(3, 0) = _w;
    }

    auto Quaternion::Xi() -> Eigen::Matrix<double, 4, 3> {
        Eigen::Vector3d q;
        q(0, 0) = m_q.x();
        q(1, 0) = m_q.y();
        q(2, 0) = m_q.z();
        double q4 = m_q.w();
        auto Q_x = skew(q);
        Eigen::Matrix<double, 4, 3> Xi;
        Xi.setZero();
        Xi.topLeftCorner(3, 3) = q4 * Eigen::Matrix3d::Identity() + Q_x;
        Xi.row(3) = -q;
        return Xi;
    }

    auto Quaternion::Psi() -> Eigen::Matrix<double, 4, 3> {
        Eigen::Vector3d q;
        q(0, 0) = m_q.x();
        q(1, 0) = m_q.y();
        q(2, 0) = m_q.z();
        double q4 = m_q.w();
        auto Q_x = skew(q);
        Eigen::Matrix<double, 4, 3> Psi;
        Psi.setZero();
        Psi.topLeftCorner(3, 3) = q4 * Eigen::Matrix3d::Identity() - Q_x;
        Psi.row(3) = -q.transpose();
        return Psi;
    }

    Quaternion::Quaternion(const Quaternion &_other) {
        m_q = _other.getQuaternion();
    }

    Quaternion::Quaternion(const double &_x, const double &_y, const double &_z, const double &_w)
            : AngleRepresentationInterface() {
        setQuaternion(_x, _y, _z, _w);
    }

    Quaternion::Quaternion(const Eigen::Matrix<double, 3, 3> &_m) : AngleRepresentationInterface() {
        m_q = fromDCMtoQuaternion(_m);
    }

    Quaternion::Quaternion(const Eigen::Matrix<double, 4, 1> &_quat) : AngleRepresentationInterface(), m_q(_quat) {}

    Quaternion &Quaternion::operator=(const Quaternion &_other) {
        m_q = _other.getQuaternion();
    }




    std::ostream &operator<<(std::ostream &os, const Quaternion &_inst) {
        os << "quat { x : " << _inst.x() << ", y : " << _inst.y() << ", z : " << _inst.z() << ", w : " << _inst.w()
           << " };";
        return os;
    }

    void Quaternion::imagine() {
        m_q = imaginaryEigenQuat();
    }

    auto Quaternion::imaginaryEigenQuat() -> Eigen::Matrix<double, 4, 1> {
        return Eigen::Matrix<double, 4, 1>(x(), y(), z(), 0);
    }

    auto Quaternion::imaginaryQuaternion() -> Quaternion {
        return Quaternion(imaginaryQuaternion());
    }


    auto Quaternion::inversedEigenQuat() -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double, 4, 1> temp = m_q;
        temp(1, 0) *= -1;
        temp(2, 0) *= -1;
        temp(3, 0) *= -1;
        return temp;
    }

    auto Quaternion::inversedQuaternion() -> Quaternion {
        return Quaternion(inversedEigenQuat());
    }

    void Quaternion::inverse() {
        m_q(1, 0) *= -1;
        m_q(2, 0) *= -1;
        m_q(3, 0) *= -1;
    }

}