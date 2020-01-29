#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>

#include "Vec.h"
#include "Quat.h"

class Tform : public Eigen::Matrix4d
{
public:
    template <typename Derived>
    using BaseType = Eigen::EigenBase<Derived>;

    template <typename Derived>
    using QuatType = Eigen::QuaternionBase<Derived>;

    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix4d = Eigen::Matrix4d;
    using Quaterniond = Eigen::Quaterniond;

    Tform(void) : Matrix4d() {}

    // This constructor allows you to construct Tform from 4 by 4 Eigen matrices
    template <typename OtherDerived>
    Tform(const BaseType<OtherDerived> &other)
        : Matrix4d(other)
    {
    }

    // ctor from ROS geometry message
    Tform(const geometry_msgs::Pose &pose)
    {
        this->block<3, 1>(0, 3) = Vec(pose.position);
        this->block<3, 3>(0, 0) = Quat(pose.orientation).toRotationMatrix();
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    Tform(const Vector3d vec, const Quaterniond &quat)
    {
        this->block<3, 1>(0, 3) = vec;
        this->block<3, 3>(0, 0) = Quat(quat).toRotationMatrix();
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    template <typename OtherDerived>
    Tform &operator=(const BaseType<OtherDerived> &other)
    {
        this->Matrix4d::operator=(other);
        return *this;
    }

    inline Tform inverse(void)
    {
        Tform inv;
        inv.block<3, 1>(0, 3) = -this->rotation().transpose()*this->translation();
        inv.block<3, 3>(0, 0) = this->rotation().transpose();
        inv.block<1, 4>(3, 0) << 0, 0, 0, 1;
        return inv;
    }

    inline Matrix3d rotation(void) { return this->block<3, 3>(0, 0); }

    inline Vector3d translation(void) { return this->block<3, 1>(0, 3); }

    geometry_msgs::Pose toMsgsPose(void)
    {
        geometry_msgs::Pose ros_pose;
        ros_pose.orientation = Quat(this->block<3, 3>(0, 0)).toMsgsQuat();
        ros_pose.position = Vec(this->block<3, 1>(0, 3)).toMsgsPoint();
        return ros_pose;
    }
};