#ifndef QUAT_H
#define QUAT_H
#include <cmath>
#include <Eigen/Geometry>
#ifdef HAVE_ROS
#include <geometry_msgs/Quaternion.h>
#endif
template <class Scalar>
class Quat : public Eigen::Quaternion<Scalar>
{
public:
    typedef Eigen::Quaternion<Scalar> Quaternion;

    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

    Quat(void) : Quaternion() {}

    // This constructor allows you to construct Quat from other Eigen quaternions
    template <typename OtherDerived>
    Quat(const Eigen::QuaternionBase<OtherDerived> &other) : Quaternion(other){};

    // ctor from Eigen Matrices
    template <typename OtherDerived>
    Quat(const Eigen::MatrixBase<OtherDerived> &other) : Quaternion(other){};

    //ctor from elements
    Quat(const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z) : Quaternion(w, x, y, z){};

    // This method allows you to assign Eigen quaternions to Quat
    template <typename OtherDerived>
    Quat &operator=(const Eigen::QuaternionBase<OtherDerived> &other)
    {
        this->Quaternion::operator=(other);
        return *this;
    }

    // conversion to std array
    std::array<Scalar, 4> toStdArray(void)
    {
        auto q = this->coeffsWxyz();
        return std::array<Scalar, 4>{q(0), q(1), q(2), q(3)};
    }

    // conversion to Boost array
    boost::array<Scalar, 4> toBoostArray(void)
    {
        auto q = this->coeffsWxyz();
        return boost::array<Scalar, 4>{q(0), q(1), q(2), q(3)};
    }

    Matrix3 toDCM(void) const
    {
        return this->normalized().toRotationMatrix().transpose();
    }

    Vector4 coeffsWxyz(void) const
    {
        auto q = this->Quaternion::coeffs();
        return Vector4(q(3), q(0), q(1), q(2));
    }

    // Conversion to 321-Euler angle sequence
    Vector3 toEul(void) const
    {
        auto q = this->coeffsWxyz();
        return Vector3(atan2(2.0 * (q(0) * q(1) + q(2) * q(3)), 1.0 - 2.0 * (q(1) * q(1) + q(2) * q(2))),
                       asin(2.0 * (q(0) * q(2) - q(3) * q(1))),
                       atan2(2.0 * (q(0) * q(3) + q(1) * q(2)), 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3))));
    }

    // Conversion to so3 element
    Vector3 toRotvec(void) const
    {
        auto q = this->coeffsWxyz();
        return 2 * acos(q(0)) * q.template block<3, 1>(1, 0).normalized();
    }

#ifdef HAVE_ROS
    // ctor from ROS geometry message
    Quat(const geometry_msgs::Quaternion &ros_quat) : Quaternion(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z){};

    geometry_msgs::Quaternion toMsgsQuat(void)
    {
        geometry_msgs::Quaternion ros_quat;
        Vector4 q = this->Quaternion::coeffs();
        ros_quat.w = q(3);
        ros_quat.x = q(0);
        ros_quat.y = q(1);
        ros_quat.z = q(2);

        return ros_quat;
    }
#endif
};

typedef Quat<double> Quatd;
typedef Quat<float> Quatf;

#endif