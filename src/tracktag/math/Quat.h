#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#ifndef QUAT_H
#define QUAT_H

class Quat : public Eigen::Quaterniond
{
public:
    template <typename Derived>
    using QuatType = const Eigen::QuaternionBase<Derived>;

    template <typename Derived>
    using MatType = const Eigen::MatrixBase<Derived>;

    using Quaterniond = Eigen::Quaterniond;

    Quat(void) : Quaterniond() {}
    // This constructor allows you to construct Quat from other Eigen quaternions
    template <typename OtherDerived>
    Quat(const QuatType<OtherDerived> &other) : Quaterniond(other){};

    // ctor from Eigen Matrices
    template <typename OtherDerived>
    Quat(const MatType<OtherDerived> &other) : Quaterniond(other){};

    //ctor from elements
    Quat(const double &w, const double &x, const double &y, const double &z) : Quaterniond(w, x, y, z){};

    // ctor from ROS geometry message
    Quat(const geometry_msgs::Quaternion &ros_quat) : Quaterniond(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z){};

    // This method allows you to assign Eigen quaternions to Quat
    template <typename OtherDerived>
    Quat &operator=(const QuatType<OtherDerived> &other)
    {
        this->Quaterniond::operator=(other);
        return *this;
    }
    
    geometry_msgs::Quaternion toMsgsQuat(void)
    {
        geometry_msgs::Quaternion ros_quat;
        Eigen::Vector4d QuatCoeff = this->Quaterniond::coeffs();
        ros_quat.w = QuatCoeff(3);
        ros_quat.x = QuatCoeff(0);
        ros_quat.y = QuatCoeff(1);
        ros_quat.z = QuatCoeff(2);

        return ros_quat;
    }

    Eigen::Matrix3d toDCM(void)
    {
        return this->Quaterniond::toRotationMatrix().transpose();
    }

    Eigen::Vector4d coeffs_wxyz(void)
    {
        Eigen::Vector4d QuatCoeff = this->Quaterniond::coeffs();
        Eigen::Vector4d WxyzCoeff(QuatCoeff(3), QuatCoeff(0), QuatCoeff(1), QuatCoeff(2)); 
        return WxyzCoeff;
    }
};
#endif