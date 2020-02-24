#ifndef VEC_H
#define VEC_H
#include <Eigen/Eigen>
#include "boost/array.hpp"
#ifdef HAVE_ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#endif
template <class Scalar>
class Vec : public Eigen::Matrix<Scalar, 3, 1>
{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Vec(void) : Vector3() {}
    // This constructor allows you to construct Vec from Eigen expressions
    template <typename OtherDerived>
    Vec(const Eigen::EigenBase<OtherDerived> &other) : Vector3(other) {}

    // ctor from vector components
    Vec(const Scalar &x, const Scalar &y, const Scalar &z) : Vector3(x, y, z){};

    // ctor from C-style arrays. Readers of this source code are reminded that C-style arrays in C++ is bad practice
    Vec(const Scalar (&v)[3]) : Vector3(v[0], v[1], v[2]){};

    Vec(const boost::array<Scalar, 3> &v) : Vector3(v[0], v[1], v[2]){};

    // This method allows you to assign Eigen expressions to Vec
    template <typename OtherDerived>
    Vec &operator=(const Eigen::EigenBase<OtherDerived> &other)
    {
        this->Vector3::operator=(other);
        return *this;
    }

    // conversion to std array
    std::array<Scalar, 3> toStdArray(void)
    {
        return std::array<Scalar, 3>{this->Vector3::operator()(0),
                                     this->Vector3::operator()(1),
                                     this->Vector3::operator()(2)};
    }

    // conversion to Boost array
    boost::array<Scalar, 3> toBoostArray(void)
    {
        return boost::array<Scalar, 3>{this->Vector3::operator()(0),
                                       this->Vector3::operator()(1),
                                       this->Vector3::operator()(2)};
    }
#ifdef HAVE_ROS
    // ctor from ROS geometry message point
    Vec(const geometry_msgs::Point &point) : Vector3(static_cast<Scalar>(point.x),
                                                             static_cast<Scalar>(point.y),
                                                             static_cast<Scalar>(point.z)){};

    // ctor from ROS geometry message vector3
    Vec(const geometry_msgs::Vector3 &vector3) : Vector3(vector3.x, vector3.y, vector3.z){};
    // conversion to ROS geometry message point

    geometry_msgs::Point toMsgsPoint(void)
    {
        geometry_msgs::Point point;
        point.x = static_cast<double>(this->Vector3::operator()(0));
        point.y = static_cast<double>(this->Vector3::operator()(1));
        point.z = static_cast<double>(this->Vector3::operator()(2));
        return point;
    }

    // // conversion to ROS geometry message vector3
    geometry_msgs::Vector3 toMsgsVector3(void)
    {
        geometry_msgs::Vector3 vector3;
        vector3.x = static_cast<double>(this->Vector3::operator()(0));
        vector3.y = static_cast<double>(this->Vector3::operator()(1));
        vector3.z = static_cast<double>(this->Vector3::operator()(2));

        return vector3;
    }
#endif
};

typedef Vec<float> Vecf;
typedef Vec<double> Vecd;
#endif