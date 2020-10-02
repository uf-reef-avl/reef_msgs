//
// Created by paulbuzaud on 9/17/20.
//

#include "reef_msgs/ReefQuat.h"



namespace reef_msgs{
  auto ReefQuat::eye() -> ReefQuat {
    ReefQuat instEye(0,0,0,1);
    return instEye;
  }
  auto ReefQuat::rand() -> ReefQuat {
    ReefQuat instRand(((double) std::rand() / RAND_MAX),((double) std::rand() / RAND_MAX),((double) std::rand() / RAND_MAX),((double) std::rand() / RAND_MAX));
    instRand.normalize();
    return instRand;
  }

auto ReefQuat::quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q) -> Eigen::Matrix<double,4,1> {
  double pw = p(3,0);

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d p_skew;
  p_skew = skew(p.head(3));

  Eigen::Matrix4d A;
  A.block<3,3>(0,0) = pw * I * p_skew;
  A.block<4,1>(0,3) = p;
  A.block<1,4>(3,0) = -p.transpose();
  A(3,3) = pw;

  return A*q;
}

auto ReefQuat::quatMult(Eigen::Quaterniond q , Eigen::Quaterniond p) -> Eigen::Quaterniond {

  Eigen::Vector4d p_eigen;
  p_eigen = p.coeffs();
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d q_skew;
  q_skew = skew(q.vec());

  Eigen::Matrix4d A;
  A.block<3,3>(0,0) = q.w() * I - q_skew;
  A.block<3,1>(0,3) = q.vec();
  A.block<1,3>(3,0) = -q.vec();
  A(3,3) = q.w();
  Eigen::Vector4d qp;
  qp = A*p_eigen;
  Eigen::Quaterniond q_times_p;
  q_times_p.vec() << qp(0),qp(1),qp(2);
  q_times_p.w() = qp(3);
  return q_times_p;
}

auto ReefQuat::quaternionMultiplication(Eigen::Quaterniond p , Eigen::Quaterniond q) -> Eigen::Quaterniond {
  Eigen::Matrix<double,4,1> p_temp;
  Eigen::Matrix<double,4,1> q_temp;
  Eigen::Matrix<double,4,1> multiplicant;

  p_temp = quat2vector(p);
  q_temp = quat2vector(q);

  multiplicant = quaternionMultiplication(p_temp,q_temp);

  return vector2quat(multiplicant);
}

auto ReefQuat::quaternionToRotation(Eigen::Quaterniond q) -> Eigen::Matrix3d
{
  Eigen::Matrix3d I;
  I.setIdentity();

  Eigen::Matrix3d skew_mat = skew(Eigen::Vector3d(q.x(), q.y(), q.z()));
  Eigen::Matrix3d rotation_matrix = I - 2 * q.w() * skew_mat + 2 * skew_mat * skew_mat;

  return rotation_matrix;
}

auto ReefQuat::quaternionToRotation(geometry_msgs::Quaternion q) -> Eigen::Matrix3d {
  Eigen::Quaterniond q_temp;
  q_temp.x() = q.x;
  q_temp.y() = q.y;
  q_temp.z() = q.z;
  q_temp.w() = q.w;

  return quaternionToRotation(q_temp);
}
auto ReefQuat::toRotation() -> Eigen::Matrix3d {
  Eigen::Quaterniond q_temp;
  q_temp.x() = m_q.x();
  q_temp.y() = m_q.y();
  q_temp.z() = m_q.z();
  q_temp.w() = m_q.w();

  return quaternionToRotation(q_temp);
};
    auto ReefQuat::toDCM() -> Eigen::Matrix3d {
        Eigen::Matrix3d m = this->Xi().transpose() * this->Psi();
        return m;
    };
    auto ReefQuat::toRM() -> Eigen::Matrix3d {
        Eigen::Matrix3d m = this->toDCM().transpose();
        return m;
    }

    auto ReefQuat::fromDCMtoQuaternion(const Eigen::Matrix<double, 3, 3> &_m) -> Eigen::Matrix<double, 4, 1> {
        auto gamma = _m.trace();
        double w2 = (1 + gamma)/4;
        Eigen::Matrix<double, 3, 1> Ckk = _m.diagonal();
        Eigen::Matrix<double, 3, 1> quat2temp = (Eigen::Matrix<double,3,1>::Ones() + 2 * Ckk - (gamma * Eigen::Matrix<double,3,1>::Ones())) / 4.;
        Eigen::Matrix<double, 4, 1> quat2;
        quat2 << quat2temp(0,0), quat2temp(1,0),quat2temp(2,0), w2;
        int maxIndex = 0;
        double max = quat2[0];
        for(int i = 0; i < 4; i++) {
            if(quat2(i,0) > max) {
                max = quat2(i,0);
                maxIndex = i;
            }
        }
        Eigen::Matrix<double, 4, 1> q;
        q.setZero();
        q(maxIndex,0) = sqrt(quat2(maxIndex,0));
        double d = 4. * q[maxIndex];
        if (maxIndex == 3) {
            q(0,0) = (_m(1,2) - _m(2,1))/d;
            q(1,0) = (_m(2,0) - _m(0,2))/d;
            q(2,0) = (_m(0,1) - _m(1,0))/d;
        }else if (maxIndex == 0) {
            q(3,0) = (_m(1,2) - _m(2,1))/d;
            q(1,0) = (_m(0,1) + _m(1,0))/d;
            q(2,0) = (_m(2,0) + _m(0,2))/d;
        }else if (maxIndex == 1) {
            q(3,0) = (_m(2,0) - _m(0,2))/d;
            q(0,0) = (_m(0,1) + _m(1,0))/d;
            q(2,0) = (_m(1,2) + _m(2,1))/d;
        }else if (maxIndex == 2) {
            q(3,0) = (_m(0,1) - _m(1,0))/d;
            q(0,0) = (_m(2,0) + _m(0,2))/d;
            q(1,0) = (_m(1,2) + _m(2,1))/d;
        }
        q.normalize();

        return q;
    }

    auto ReefQuat::fromAngleAxistoQuaternion(const double &_angle,
                                             const Eigen::Matrix<double, 3, 1> &_axis) -> Eigen::Matrix<double, 4, 1> {
        Eigen::Matrix<double,4,1> quat;
        quat.setZero();
        Eigen::Matrix<double, 3, 1> q = sin(_angle/2.) * _axis.normalized();
        quat.block<3,1>(0,0) << q;
        quat(3,0) = cos(_angle/2.);
        return quat;
    }

  double ReefQuat::norm() {
    return m_q.norm();
  }

  void ReefQuat::normalize() {
     m_q.normalize();
  }

  void ReefQuat::setQuaternion(const double &_x, const double &_y, const double &_z, const double &_w) {
    m_q(0,0) = _x;
    m_q(1,0) = _y;
    m_q(2,0) = _z;
    m_q(3,0) = _w;
  }
  auto ReefQuat::getQuaternion() const -> const Eigen::Matrix<double, 4, 1> & {
      return m_q;
  }
  auto ReefQuat::x() const -> double {
  return m_q(0,0);
  }
  void ReefQuat::setX(const double &_x) {
    m_q(0,0) = _x;
  }
  auto ReefQuat::y() const ->  double {
    return m_q(1,0);
  }
  void ReefQuat::setY(const double &_y) {
    m_q(1,0) = _y;
  }
  auto ReefQuat::z() const ->  double {
    return m_q(2,0);
  }
  void ReefQuat::setZ(const double &_z) {
    m_q(2,0) = _z;
  }
  auto ReefQuat::w() const -> double {
    return m_q(3,0);
  }
  void ReefQuat::setW(const double &_w) {
    m_q(3,0) = _w;
  }
  auto ReefQuat::Xi() -> Eigen::Matrix<double, 4, 3> {
      Eigen::Vector3d q;
      q(0,0) =  m_q.x();
      q(1,0) =  m_q.y();
      q(2,0) =  m_q.z();
    double q4 = m_q.w();
    auto Q_x = skew(q);
    Eigen::Matrix<double, 4, 3> Xi;
    Xi.setZero();
    Xi.topLeftCorner(3,3) = q4 *  Eigen::Matrix3d::Identity() + Q_x ;
      Xi.row(3) = -q;
    return Xi;
  }
  auto ReefQuat::Psi() -> Eigen::Matrix<double, 4, 3> {
      Eigen::Vector3d q;
      q(0,0) =  m_q.x();
      q(1,0) =  m_q.y();
      q(2,0) =  m_q.z();
      double q4 = m_q.w();
      auto Q_x = skew(q);
      Eigen::Matrix<double, 4, 3> Psi;
      Psi.setZero();
      Psi.topLeftCorner(3,3) = q4 *  Eigen::Matrix3d::Identity() - Q_x ;
      Psi.row(3) = - q.transpose();
      return Psi;
  }
  ReefQuat::ReefQuat(const ReefQuat &_other) {
    m_q = _other.getQuaternion();
  }

  ReefQuat::ReefQuat(const double &_x, const double &_y, const double &_z, const double &_w):AngleRepresentationInterface(){
    setQuaternion(_x,_y,_z,_w);
  }
    ReefQuat::ReefQuat(const Eigen::Matrix<double,3,3> &_m):AngleRepresentationInterface(){
        m_q = fromDCMtoQuaternion(_m);
    }
  ReefQuat::ReefQuat(const Eigen::Matrix<double, 4, 1> &_quat):AngleRepresentationInterface(), m_q(_quat) {}
  ReefQuat & ReefQuat::operator=(const ReefQuat & _other) {
    m_q = _other.getQuaternion();
  }
  ReefQuat & ReefQuat::operator*(const ReefQuat & _other) {
    m_q = quaternionMultiplication(m_q,_other.getQuaternion());
  }
  std::ostream & operator<<(std::ostream &os, const ReefQuat &_inst) {
    os << "quat { x : " << _inst.x() << ", y : " << _inst.y() <<", z : " << _inst.z() <<", w : " << _inst.w()<<" };" ;
    return os;
  }
  void ReefQuat::imagine(){
    m_q = imaginaryEigenQuat();
  }
  auto ReefQuat::imaginaryEigenQuat() -> Eigen::Matrix<double, 4, 1> {
    return Eigen::Matrix<double, 4, 1>(x(),y(),z(),0);
  }
    auto ReefQuat::imaginaryReefQuat() -> ReefQuat  {
        return ReefQuat(imaginaryReefQuat());
    }


  auto ReefQuat::inversedEigenQuat() -> Eigen::Matrix<double, 4, 1> {
    Eigen::Matrix<double, 4, 1> temp = m_q;
    temp(1,0) *= -1;
    temp(2,0) *= -1;
    temp(3,0) *= -1;
    return temp;
  }
  auto ReefQuat::inversedReefQuat() -> ReefQuat {
    return ReefQuat(inversedEigenQuat());
  }
  void ReefQuat::inverse() {
    m_q(1,0) *= -1;
    m_q(2,0) *= -1;
    m_q(3,0) *= -1;
  }
}