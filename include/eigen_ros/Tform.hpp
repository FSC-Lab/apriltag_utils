#ifndef TFORM_H
#define TFORM_H
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#ifdef HAVE_ROS
#include <geometry_msgs/Pose.h>
#endif

#ifdef HAVE_APRILTAG
#include "apriltag_pose.h"
#endif

#include "Vec.hpp"
#include "Quat.hpp"

template <class Scalar>
class Tform : public Eigen::Matrix<Scalar, 4, 4>
{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
    
    using Quaternion = Eigen::Quaternion<Scalar>;

    // Default ctor sets transform matrix to identity
    Tform(void) : Matrix4() { this->setIdentity(); }

    // ctor from 4 by 4 Eigen matrices
    template <typename OtherDerived>
    Tform(const Eigen::EigenBase<OtherDerived> &other) : Matrix4(other) { this->template block<1, 4>(3, 0) << 0, 0, 0, 1; }

    // ctor from a vector and a quaternion
    Tform(const Vector3 &vec, const Quaternion &quat)
    {
        this->template block<3, 1>(0, 3) = Vec<Scalar>(vec);
        this->template block<3, 3>(0, 0) = Quat<Scalar>(quat).normalized().toRotationMatrix();
        this->template block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    // ctor from a DCM and a quaternion, be mindful of DCM convention!
    Tform(const Vector3 &vec, const Matrix3 &mat)
    {
        this->template block<3, 1>(0, 3) = vec;
        this->template block<3, 3>(0, 0) = mat;
        this->template block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    // assignment operator
    template <typename OtherDerived>
    Tform &operator=(const Eigen::EigenBase<OtherDerived> &other)
    {
        this->Matrix4::operator=(other);
        return *this;
    }

    Matrix3 rotation(void) const { return this->template block<3, 3>(0, 0); }

    Vector3 translation(void) const { return this->template block<3, 1>(0, 3); }

    Tform inverse(void) const
    {
        Tform inv;
        inv.template block<3, 1>(0, 3) = -this->rotation().transpose() * this->translation();
        inv.template block<3, 3>(0, 0) = this->rotation().transpose();
        inv.template block<1, 4>(3, 0) << 0, 0, 0, 1;
        return inv;
    }

#ifdef HAVE_ROS
    // ctor from ROS geometry message pose
    Tform(const geometry_msgs::Pose &pose)
    {
        this->template block<3, 1>(0, 3) = Vec<Scalar>(pose.position);
        this->template block<3, 3>(0, 0) = Quat<Scalar>(pose.orientation).toRotationMatrix();
        this->template block<1, 4>(3, 0) << 0, 0, 0, 1;
    }

    geometry_msgs::Pose toMsgsPose(void) const
    {
        geometry_msgs::Pose ros_pose;
        ros_pose.orientation = Quat<Scalar>(this->template block<3, 3>(0, 0)).toMsgsQuat();
        ros_pose.position = Vec<Scalar>(this->template block<3, 1>(0, 3)).toMsgsPoint();
        return ros_pose;
    }
#endif

#ifdef HAVE_APRILTAG
    // ctor from apriltag pose struct
    Tform(const apriltag_pose_t &pose)
    {
        this->template block<3, 1>(0, 3) << pose.t->data[0], pose.t->data[1], pose.t->data[2];
        this->template block<3, 3>(0, 0) << pose.R->data[0], pose.R->data[1], pose.R->data[2],
            pose.R->data[3], pose.R->data[4], pose.R->data[5],
            pose.R->data[6], pose.R->data[7], pose.R->data[8];
        this->template block<1, 4>(3, 0) << 0, 0, 0, 1;
    }
#endif
};

typedef Tform<float> Tformf;
typedef Tform<double> Tformd;

#endif