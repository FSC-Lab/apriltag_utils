#ifndef VEC_H
#define VEC_H
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

class Vec : public Eigen::Vector3d
{
public:
    template <typename Derived>
    using BaseType = Eigen::EigenBase<Derived>;

    using Vector3d = Eigen::Vector3d;

    Vec(void) : Vector3d() {}
    // This constructor allows you to construct Vec from Eigen expressions
    template <typename OtherDerived>
    Vec(const BaseType<OtherDerived> &other)
        : Vector3d(other)
    {
    }

    // ctor from vector components
    Vec(const double &x, const double &y, const double &z) : Vector3d(x, y, z){};

    // ctor from ROS geometry message point
    Vec(const geometry_msgs::Point &point) : Vector3d(point.x, point.y, point.z){};

    // ctor from ROS geometry message vector3
    Vec(const geometry_msgs::Vector3 &vector3) : Vector3d(vector3.x, vector3.y, vector3.z){};

    // This method allows you to assign Eigen expressions to Vec
    template <typename OtherDerived>
    Vec &operator=(const BaseType<OtherDerived> &other)
    {
        this->Vector3d::operator=(other);
        return *this;
    }

    // conversion to ROS geometry message point
    geometry_msgs::Point toMsgsPoint(void)
    {
        geometry_msgs::Point point;
        point.x = this->Vector3d::operator()(0);
        point.y = this->Vector3d::operator()(1);
        point.z = this->Vector3d::operator()(2);

        return point;
    }

    // conversion to ROS geometry message vector3
    geometry_msgs::Vector3 toMsgsVector3(void)
    {
        geometry_msgs::Vector3 vector3;
        vector3.x = this->Vector3d::operator()(0);
        vector3.y = this->Vector3d::operator()(1);
        vector3.z = this->Vector3d::operator()(2);

        return vector3;
    }
};
#endif