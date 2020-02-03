#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "common.hpp"
#include <ros/ros.h>
#include <apriltag_utils/Mocap.h>
#include "geometry_msgs/PoseStamped.h"
#include <queue>

using namespace geometry_msgs;
using namespace message_filters;
using namespace std;

using Mocap = apriltag_utils::Mocap;

class Sampler
{
public:
    queue<Mocap> *T_p_i_k;
    queue<Mocap> *T_v_i_k;

    PoseStamped T_p_v_0;
    double t_k;

    Sampler() : t_k(0),
                sync(sub_payload, sub_vehicle, 100)
    {
        T_p_i_k = new queue<Mocap>;
        T_v_i_k = new queue<Mocap>;
        ros::NodeHandle nh;

        payload_pub = nh.advertise<PoseStamped>("/sample/abs_pose_v", 10);
        vehicle_pub = nh.advertise<PoseStamped>("/sample/abs_pose_p", 10);
        camera_pub = nh.advertise<PoseStamped>("/sample/rel_pose_p", 10);

        sub_payload.subscribe(nh, "/mocap/Payload", 10);
        sub_vehicle.subscribe(nh, "/mocap/UAV2", 10);
        sync.registerCallback(boost::bind(&Sampler::doQueue, this, _1, _2));
        sub_camera = nh.subscribe<PoseStamped>("/RPi_camera_v2/apriltag/pose", 10, &Sampler::doPublish, this);

        std::cout << "Attempting to construct class" << std::endl;
    }

    ~Sampler()
    {
        delete T_p_i_k;
        delete T_v_i_k;
    }

    void doQueue(const Mocap::ConstPtr &payload_msg, const Mocap::ConstPtr &vehicle_msg)
    {
        cout << "Queue" << endl;
        Mocap payload = *payload_msg;
        Mocap vehicle = *vehicle_msg;
        T_p_i_k->push(payload);
        T_v_i_k->push(vehicle);
    }

    void doPublish(const PoseStamped::ConstPtr &msg)
    {
        std::cout << "Synchronizing" << std::endl;
        PoseStamped T_p_v_1 = *msg; // Pose at interpolation endpoint
        PoseStamped T_p_v;          // Target Pose
        Vec r_p_v;
        while (!T_p_i_k->empty() && !T_v_i_k->empty())
        {
            PoseStamped T_p_i = Mocap2Pose(T_p_i_k->front());
            PoseStamped T_v_i = Mocap2Pose(T_v_i_k->front());

            double t1 = T_p_v_1.header.stamp.toSec();
            double t0 = T_p_v_0.header.stamp.toSec();
            double tx = T_p_i.header.stamp.toSec();

            Vec r_p_v_1(T_p_v_1.pose.position);
            Vec r_p_v_0(T_p_v_0.pose.position);

            r_p_v = (r_p_v_1 - r_p_v_0) / (t1 - t0) * (tx - t0) + r_p_v_0;

            T_p_v.pose.position = r_p_v.toMsgsPoint();

            vehicle_pub.publish(T_v_i);
            payload_pub.publish(T_p_i);
            camera_pub.publish(T_p_v);

            T_p_i_k->pop();
            T_v_i_k->pop();
        }
        r_p_v << 0, 0, 0;
        T_p_v_0 = T_p_v_1;
        t_k = T_p_v_0.header.stamp.toSec();
    }

    static PoseStamped Mocap2Pose(Mocap mocap)
    {
        PoseStamped pose;
        pose.header.stamp = mocap.header.stamp;

        pose.pose.position.x = static_cast<double>(mocap.position[0]);
        pose.pose.position.y = static_cast<double>(mocap.position[1]);
        pose.pose.position.z = static_cast<double>(mocap.position[2]);

        pose.pose.orientation.w = static_cast<double>(mocap.quaternion[0]);
        pose.pose.orientation.x = static_cast<double>(mocap.quaternion[1]);
        pose.pose.orientation.y = static_cast<double>(mocap.quaternion[2]);
        pose.pose.orientation.z = static_cast<double>(mocap.quaternion[3]);

        return pose;
        }

private:
    ros::Publisher payload_pub;
    ros::Publisher vehicle_pub;
    ros::Publisher camera_pub;

    message_filters::Subscriber<Mocap> sub_payload;
    message_filters::Subscriber<Mocap> sub_vehicle;

    TimeSynchronizer<Mocap, Mocap> sync;

    ros::Subscriber sub_camera;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sampling_node");
    ros::start();

    ros::Rate loop_rate(10);
    Sampler sampler;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}