#include <chrono>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>

#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include "unistd.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace imt = image_transport;
namespace cim = camera_info_manager;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detector");

    ros::NodeHandle nh;
    imt::ImageTransport it(nh);
    imt::Publisher camera_image_publisher = it.advertise("/camera/image_rect", 1);

    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);

    sensor_msgs::CameraInfo c_info;
    sensor_msgs::ImagePtr msg;

    cv::Mat frame, gray, un_gray;

    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")

        ("camera_name,n", po::value<std::string>()->default_value("camera"), "Name of camera")

            ("camera_info_url", po::value<std::string>(), "Location of calibration file")

                ("capture_width", po::value<int>()->default_value(1280), "Width of Captured Video in pixels")

                    ("capture_height", po::value<int>()->default_value(720), "Height of Captured Video in pixels")

                        ("display_width", po::value<int>()->default_value(1280), "Width of Displayed Video in pixels")

                            ("display_height", po::value<int>()->default_value(720), "Height of Displayed Video in pixels")

                                ("framerate", po::value<int>()->default_value(30), "Frame Rate")

                                    ("flip_method", po::value<int>()->default_value(30), "Video rotation");

    po::variables_map vm;

    po::store(po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::default_style | po::command_line_style::allow_long_disguise).run(), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 0;
    }

    std::string camera_name = vm["camera_name"].as<std::string>();

    std::string camera_info_url = "file://" + ros::package::getPath("apriltag_utils") + "/camera_info/" + camera_name + ".yaml";

    if (vm.count("camera_info_url"))
        camera_info_url = "file://" + fs::absolute(fs::path(vm["camera_info_url"].as<std::string>())).generic_string();

    cim::CameraInfoManager c_info_man(nh, camera_name, camera_info_url);
    // Initialize Camera through CSI
    std::string capture_width = std::to_string(vm["capture_width"].as<int>());
    std::string capture_height = std::to_string(vm["capture_height"].as<int>());
    std::string display_width = std::to_string(vm["display_width"].as<int>());
    std::string display_height = std::to_string(vm["display_height"].as<int>());
    std::string framerate = std::to_string(vm["framerate"].as<int>());
    std::string flip_method = std::to_string(vm["flip_method"].as<int>());

    std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + capture_width + ", height=(int)" +
                           capture_height + ", format=(string)NV12, framerate=(fraction)" + framerate +
                           "/1 ! nvvidconv flip-method=" + flip_method + " ! video/x-raw, width=(int)" + display_width + ", height=(int)" +
                           display_height + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!c_info_man.loadCameraInfo(camera_info_url))
    {
        ROS_INFO("Calibration file missing. Camera not calibrated");
    }
    else
    {
        c_info = c_info_man.getCameraInfo();
        ROS_INFO("Camera successfully calibrated from default file");
    }

    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera.");
        return 1;
    }

    ROS_INFO("Using pipeline: %s", pipeline.c_str());
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
            msg->header.stamp = ros::Time::now();
            camera_image_publisher.publish(msg);
            camera_info_pub.publish(c_info);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
