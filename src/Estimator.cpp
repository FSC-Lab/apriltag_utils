//This is a file that subscribe pose of apriltag and 
//the do the Estimator to compensate and improve accuracy.

#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include "../include/Filter.hpp"

int main(int argc , char** argv)
{
    ros::init(argc, argv, "pose_filter");
    ros::start(); // ros inilization

    Eigen::Matrix<double, 6, 6> P0, Q, R;
    P0.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    Q.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    R.diagonal() << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05;

    Eigen::Matrix<double, 6, 6> H;
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    Filter Estimator( P0, Q, R);

    ros::Rate loop_rate(100);

    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("rpicamerav2/apriltag/pose", 10, &Filter::receivedata, &Estimator);
    ros::Publisher pose_publisher = nh.advertise<nav_msgs::Odometry>("/uav2/px4_command/visualmeasurement", 10);

    while (ros::ok())
    {
        if (Estimator.ini == true)
        {
            Estimator.predict();
        }
        ros::spinOnce();
        Estimator.publishpose(pose_publisher);

        loop_rate.sleep();
    }

}



