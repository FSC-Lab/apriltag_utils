#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include "common.hpp"
#include "unistd.h"

extern "C"
{
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

class Detector
{
public:
    Detector(int capture_width,
             int capture_height,
             int display_width,
             int display_height,
             int framerate,
             int flip_method);

    ~Detector();

    bool init(apriltag_detection_info_t &pose_est_info,
              std::string family = "tag36h11",
              double decimate = 2.0,
              double blur = 0,
              int threads = 1,
              bool debug = false,
              bool refine_edges = true);

    void get_image(bool publish);
    void get_pose();
    bool load_parameters(std::string filepath);

    Tformf T_p_c;
    Tformf T_iden;
    std::string pipeline_;
    std::string family_;

    geometry_msgs::PoseStamped pose_msg;

private:
    int capture_width;
    int capture_height;
    int display_width;
    int display_height;
    int framerate;
    int flip_method;

    cv::VideoCapture cap;

    cv::Mat frame, gray, un_gray;

    apriltag_detection_info_t *info;
    apriltag_detection_t *det;
    apriltag_family_t *tf = NULL;
    apriltag_detector_t *td;

    ros::Publisher pose_pub;
    image_transport::Publisher image_pub;
    sensor_msgs::ImagePtr msg;
};