#ifndef TFORM_H
#define TFORM_H
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>

#ifdef HAVE_APRILTAG
#include "apriltag_pose.h"
#endif

#include "Vec.hpp"
#include "Quat.hpp"

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

    // Default ctor sets transform matrix to identity
    explicit Tform(void) : Matrix4d() { this->setIdentity(); }

    // ctor from 4 by 4 Eigen matrices
    template <typename OtherDerived>
    explicit Tform(const BaseType<OtherDerived> &other)
        : Matrix4d(other)
    {
    }

    // ctor from ROS geometry message pose
    Tform(const geometry_msgs::Pose &pose)
    {
        this->block<3, 1>(0, 3) = Vec(pose.position);
        this->block<3, 3>(0, 0) = Quat(pose.orientation).toRotationMatrix();
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    // ctor from a vector and a quaternion
    Tform(const Vector3d &vec, const Quaterniond &quat)
    {
        this->block<3, 1>(0, 3) = Vec(vec);
        this->block<3, 3>(0, 0) = Quat(quat).toRotationMatrix();
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    // ctor from a DCM and a quaternion, be mindful of DCM convention!
    Tform(const Vector3d &vec, const Matrix3d &mat)
    {
        this->block<3, 1>(0, 3) = vec;
        this->block<3, 3>(0, 0) = mat;
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    // ctor from apriltag pose struct
#ifdef HAVE_APRILTAG
    Tform(const apriltag_pose_t &pose)
    {
        this->block<3, 1>(0, 3) << pose.t->data[0], pose.t->data[1], pose.t->data[2];
        this->block<3, 3>(0, 0) << pose.R->data[0], pose.R->data[1], pose.R->data[2],
            pose.R->data[3], pose.R->data[4], pose.R->data[5],
            pose.R->data[6], pose.R->data[7], pose.R->data[8];
        this->block<1, 4>(3, 0) << 0, 0, 0, 1;
    }
#endif

    template <typename OtherDerived>
    Tform &operator=(const BaseType<OtherDerived> &other)
    {
        this->Matrix4d::operator=(other);
        return *this;
    }

    Tform inverse(void)
    {
        Tform inv;
        inv.block<3, 1>(0, 3) = -this->rotation().transpose() * this->translation();
        inv.block<3, 3>(0, 0) = this->rotation().transpose();
        inv.block<1, 4>(3, 0) << 0, 0, 0, 1;
        return inv;
    }

    Matrix3d rotation(void) { return this->block<3, 3>(0, 0); }

    Vector3d translation(void) { return this->block<3, 1>(0, 3); }

    geometry_msgs::Pose toMsgsPose(void)
    {
        geometry_msgs::Pose ros_pose;
        ros_pose.orientation = Quat(this->block<3, 3>(0, 0)).toMsgsQuat();
        ros_pose.position = Vec(this->block<3, 1>(0, 3)).toMsgsPoint();
        return ros_pose;
    }
};
#endif