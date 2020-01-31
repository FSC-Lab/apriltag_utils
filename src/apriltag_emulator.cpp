#include <iostream>
#include <mutex>
#include <map>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "common.hpp"

#include "../include/math/Quat.h"
#include "../include/math/Vec.h"
#include "../include/math/Tform.h"

#define TFORM_USE 1

//Declaration
void GetPayloadPose(const nav_msgs::Odometry::ConstPtr &payloadmsg, const nav_msgs::Odometry::ConstPtr &uavmsg);
void FlagDetect(const std_msgs::String::ConstPtr &msg);
void initros();
bool Loss_Flag = 1;
bool pub_flag = 0;
std::vector<std::pair<double, Tform>> T_pk_vk;
Eigen::Vector3d CalculateVelocityFromPose();
nav_msgs::Odometry pose_msg;
int count = 0;

//Definition
void GetPayloadPose(const nav_msgs::Odometry::ConstPtr &payloadmsg, const nav_msgs::Odometry::ConstPtr &uavmsg)
{
    nav_msgs::Odometry payload_msg = *payloadmsg;
    nav_msgs::Odometry uav_msg = *uavmsg;

    double payload_time;
#if TFORM_USE
    Tform T_p_i(payload_msg.pose.pose);
    Tform T_v_i(uav_msg.pose.pose);

#else
    //obtain payload state
    // REWRITE NOTE:
    Quat q_payload(payload_msg.pose.pose.orientation);
    Eigen::Matrix3d C_p_c = q_payload.toRotationMatrix();
    Eigen::Vector3d r_p_c;
    r_p_c << payload_msg.pose.pose.position.x, payload_msg.pose.pose.position.y, payload_msg.pose.pose.position.z;
    Eigen::Matrix4d T_p_i(= Eigen::MatrixXd::Identity(4, 4);
    T_p_i(block<3, 3>(0, 0) = C_p_c;
    T_p_i(block<3, 1>(0, 3) = r_p_c;

    //obtain UAV state
    Quat q_uav(uav_msg.pose.pose.orientation);
    Eigen::Matrix3d uav_R = q_uav.toRotationMatrix();
    Eigen::Vector3d uav_t;
    uav_t << uav_msg.pose.pose.position.x, uav_msg.pose.pose.position.y, uav_msg.pose.pose.position.z;
    Eigen::Matrix4d T_v_i = Eigen::MatrixXd::Identity(4, 4);
    T_v_i.block<3, 3>(0, 0) = uav_R;
    T_v_i.block<3, 1>(0, 3) = uav_t;
#endif
    //obtain relative pose
    payload_time = payload_msg.header.stamp.toSec();
    Tform T_p_v = Tform(T_p_i * T_v_i.inverse());
    T_pk_vk.push_back(std::make_pair(payload_time, T_p_v));

    if (sizeof(T_pk_vk) > 10)
    {
        Vec relative_velocity = CalculateVelocityFromPose();
        pose_msg.header = payload_msg.header;

#if TFORM_USE
        //generate relative msg
        pose_msg.pose.pose = T_p_v.toMsgsPose();
#else
        Eigen::Matrix3d relative_rotation = T_p_v.block<3, 3>(0, 0);
        Quat q_relative(relative_rotation);
        pose_msg.pose.pose.position.x = T_p_v(0, 3);
        pose_msg.pose.pose.position.y = T_p_v(1, 3);
        pose_msg.pose.pose.position.z = T_p_v(2, 3);
        pose_msg.pose.pose.orientation = q_relative.toMsgsQuat();
#endif
        pose_msg.pose.covariance[1] = 0;
        pose_msg.twist.twist.linear = relative_velocity.toMsgsVector3();
        pub_flag = 1;
        // std::cout <<T_p_v<< "done" << count++ <<"\t header is" <<payload_time <<std::endl;
    }

    if (Loss_Flag == 1)
    {
        pose_msg.pose.covariance[1] = 1;
    }
}

void FlagDetect(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "on")
    {
        Loss_Flag = 0;
    }
    else if (msg->data == "off")
    {
        Loss_Flag = 1;
    }
    else
        ;
}

Eigen::Vector3d CalculateVelocityFromPose()
{
    double T_k = T_pk_vk.back().first - T_pk_vk.rbegin()[1].first;
    return (T_pk_vk.back().second - T_pk_vk[T_pk_vk.size() - 2].second).block<3, 1>(0, 3) / T_k;
}

void initros()
{
    static ros::NodeHandle nh;
}

//Main Function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_emulate");
    ros::start();
    ros::NodeHandle nh;
    ros::Publisher pose_publisher = nh.advertise<nav_msgs::Odometry>("/uav2/px4_command/visualmeasurement", 10);

    message_filters::Subscriber<nav_msgs::Odometry> left_sub(nh, "/gazebo_ground_truth_Payload", 1);
    message_filters::Subscriber<nav_msgs::Odometry> right_sub(nh, "/gazebo_ground_truth_UAV2", 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&GetPayloadPose, _1, _2));
    while (ros::ok())
    {
        ros::spinOnce();
        if (pub_flag == 1)
        {
            pose_publisher.publish(pose_msg);
            pub_flag = 0;
        }
    }
}
