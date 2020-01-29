#include <ros/ros.h>
#include "math/Quat.h"
#include "math/Vec.h"
#include <eigen_conversions/eigen_msg.h>
#include <chrono>
using namespace std::chrono;

#define TEST1 0
#define TEST2 1
int main()
{
    Quat qq;
    Eigen::Quaterniond eq;

    geometry_msgs::Quaternion g;
    geometry_msgs::Vector3 vv;

    g.w = 0.109108945117996;
    g.x = 0.327326835353989;
    g.y = 0.545544725589981;
    g.z = 0.763762615825973;

    Vec v(1, 2, 3);
    std::chrono::_V2::system_clock::duration duration;

    for (int i = 1; i < 1000; i++)
    {
        auto start = high_resolution_clock::now();
        #if TEST1
        qq = Quat(g);
        #elif TEST2
        
        tf::quaternionEigenToMsg(eq, g);
        #endif
        auto stop = high_resolution_clock::now();
        duration += duration_cast<nanoseconds>(stop - start);
    }
    auto avg = duration_cast<nanoseconds>(duration / 1000);

    std::cout << "Time taken by function: "
              << avg.count() << " microseconds" << std::endl;

    return 0;
}