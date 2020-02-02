#ifndef QUAT_H
#define QUAT_H
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>

class Quat : public Eigen::Quaterniond
{
public:
    template <typename Derived>
    using QuatType = const Eigen::QuaternionBase<Derived>;

    template <typename Derived>
    using MatType = const Eigen::MatrixBase<Derived>;

    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;
    using Vector4d = Eigen::Vector4d;
    using Matrix4d = Eigen::Matrix4d;

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
        Vector4d q = this->Quaterniond::coeffs();
        ros_quat.w = q(3);
        ros_quat.x = q(0);
        ros_quat.y = q(1);
        ros_quat.z = q(2);

        return ros_quat;
    }

    Matrix3d toDCM(void)
    {
        return this->Quaterniond::toRotationMatrix().transpose();
    }

    Vector4d coeffsWxyz(void)
    {
        Vector4d q = this->Quaterniond::coeffs();
        return Vector4d(q(3), q(0), q(1), q(2));
    }

    Vector3d toEul(void)
    {
        Vector4d q = this->coeffsWxyz();
        return Vector3d(atan2(2.0 * (q(0) * q(1) + q(2) * q(3)), 1.0 - 2.0 * (q(1) * q(1) + q(2) * q(2))),
                        asin(2.0 * (q(0) * q(2) - q(3) * q(1))),
                        atan2(2.0 * (q(0) * q(3) + q(1) * q(2)), 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3))));
    }
};
#endif