#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "common.hpp"

#include <sstream>

using namespace geometry_msgs;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "tests");

    ros::NodeHandle n;

    ros::Publisher vehicle_pub = n.advertise<PoseStamped>("/mocap/UAV2", 1000);
    ros::Publisher payload_pub = n.advertise<PoseStamped>("/mocap/Payload", 1000);
    ros::Publisher camera_pub = n.advertise<PoseStamped>("RPi_camera_v2/apriltag/pose", 100);

    ros::Rate loop_rate(10);

    /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
    int count = 0;

    Tform T_v_i;
    Tform T_p_i;
    Tform T_p_v;
    PoseStamped vehicle;
    PoseStamped payload;
    PoseStamped camera;

    vehicle.pose = T_v_i.toMsgsPose();
    payload.pose = T_p_i.toMsgsPose();
    camera.pose = T_p_v.toMsgsPose();
    int loop = 0;
    while (ros::ok())
    {
        auto time_c = ros::Time::now();

        Vec v(1.0 * loop, 1.0 * loop, 1.0 * loop);
        vehicle.pose.position = v.toMsgsPoint();
        vehicle.header.stamp = time_c;
        payload.pose.position = v.toMsgsPoint();
        payload.header.stamp = time_c;
        if (count > 10)
        {
            camera.header.stamp = time_c;
            camera.pose.position = v.toMsgsPoint();
            count = 0;
            camera_pub.publish(camera);
        }

        /**
     * This is a message object. You stuff it with data, and then publish it.
     */

        /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
        vehicle_pub.publish(vehicle);
        payload_pub.publish(payload);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
        ++loop;
    }

    return 0;
}