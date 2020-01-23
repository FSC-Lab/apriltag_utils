#ifndef filter_H
#define filter_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

// Templates for N-d matrices / arrays only!
// Template for N-d vectors, i.e. N-by-1 matrices
template <class T, int M>
using Vector = Eigen::Matrix<T, M, 1>;

template <class T, int M>
using VectorX = Vector<T, M>;

template <int N>
using VectorXd = VectorX<double, N>;

// Template for N-d matrices
template <class T, int M, int N>
using Matrix = Eigen::Matrix<T, M, N>;

template <class T, int M>
using MatrixX = Matrix<T, M, M>;

template <int N>
using MatrixXd = MatrixX<double, N>;

template <typename Derived>
using BaseType = Eigen::EigenBase<Derived>;

template <typename Derived>
using QuatType = const Eigen::QuaternionBase<Derived>;

template <typename Derived>
using MatType = const Eigen::MatrixBase<Derived>;

template <class T>
struct Constants
{
  static T epsilon() { return T(1e-8); }

  static T grav() { return T(9.80665); }

  static T pi()
  {
    return T(3.141592653589793238462643383279502884);
  }

  static T epsilonRads() { return T(3.1415926535897932384626433832e-8); }
};

class Vec : public Eigen::Vector3d
{
public:
  using Vector3d = Eigen::Vector3d;

  Vec(void) : Vector3d() {}
  // This constructor allows you to construct Vec from Eigen expressions
  template <typename OtherDerived>
  Vec(const BaseType<OtherDerived> &other)
      : Vector3d(other)
  {
  }

  // ctor from ROS geometry message
  Vec(const geometry_msgs::Point &point)
  {
    *this << point.x, point.y, point.z;
  }

  Vec(const geometry_msgs::Vector3 &vector3)
  {
    *this << vector3.x, vector3.y, vector3.z;
  }

  // This method allows you to assign Eigen expressions to Vec
  template <typename OtherDerived>
  Vec &operator=(const BaseType<OtherDerived> &other)
  {
    this->Vector3d::operator=(other);
    return *this;
  }

  geometry_msgs::Point toMsgsPoint(void)
  {
    geometry_msgs::Point point;
    point.x = this->Vector3d::operator()(0);
    point.y = this->Vector3d::operator()(1);
    point.z = this->Vector3d::operator()(2);

    return point;
  }

  geometry_msgs::Vector3 toMsgsVector3(void)
  {
    geometry_msgs::Vector3 vector3;
    vector3.x = this->Vector3d::operator()(0);
    vector3.y = this->Vector3d::operator()(1);
    vector3.z = this->Vector3d::operator()(2);

    return vector3;
  }
};

class Quat : public Eigen::Quaterniond
{
public:
  using Quaterniond = Eigen::Quaterniond;

  Quat(void) : Quaterniond() {}
  // This constructor allows you to construct Quat from Eigen expressions
  template <typename OtherDerived>
  Quat(const QuatType<OtherDerived> &other) : Quaterniond(other){};

  template <typename OtherDerived>
  Quat(const MatType<OtherDerived> &other) : Quaterniond(other){};

  // ctor from ROS geometry message
  Quat(const geometry_msgs::Quaternion &ros_quat) : Quaterniond(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z){};

  // This method allows you to assign Eigen expressions to Quat
  template <typename OtherDerived>
  Quat &operator=(const QuatType<OtherDerived> &other)
  {
    this->Quaterniond::operator=(other);
    return *this;
  }
  geometry_msgs::Quaternion toMsgsQuat(void)
  {
    geometry_msgs::Quaternion ros_quat;
    VectorXd<4> QuatCoeff = this->Quaterniond::coeffs();
    ros_quat.w = QuatCoeff(3);
    ros_quat.x = QuatCoeff(0);
    ros_quat.y = QuatCoeff(1);
    ros_quat.z = QuatCoeff(2);

    return ros_quat;
  }

  MatrixXd<3> toDCM(void)
  {
    return this->Quaterniond::toRotationMatrix().transpose();
  }
};

#endif