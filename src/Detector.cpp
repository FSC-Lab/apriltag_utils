#include <iostream>
#include <math.h>

#include "../include/Detector.hpp"

Detector::Detector(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) : capture_width(capture_width),
                                                                                                                                   capture_height(capture_height),
                                                                                                                                   display_width(display_width),
                                                                                                                                   display_height(display_height),
                                                                                                                                   framerate(framerate),
                                                                                                                                   flip_method(flip_method)
{
    pipeline_ = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
                std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
                "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
                std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    cap = cv::VideoCapture(pipeline_, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
    }
}

Detector::~Detector()
{
    apriltag_detector_destroy(td);
    if (family_ == "tag36h11")
    {
        tag36h11_destroy(tf);
    }
    else if (family_ == "tag25h9")
    {
        tag25h9_destroy(tf);
    }
    else if (family_ == "tag16h5")
    {
        tag16h5_destroy(tf);
    }
    else if (family_ == "tagCircle21h7")
    {
        tagCircle21h7_destroy(tf);
    }
    else if (family_ == "tagCircle49h12")
    {
        tagCircle49h12_destroy(tf);
    }
    else if (family_ == "tagStandard41h12")
    {
        tagStandard41h12_destroy(tf);
    }
    else if (family_ == "tagStandard52h13")
    {
        tagStandard52h13_destroy(tf);
    }
    else if (family_ == "tagCustom48h12")
    {
        tagCustom48h12_destroy(tf);
    }

    cap.release();
}

bool Detector::init(apriltag_detection_info_t &pose_est_info,
                    std::string family,
                    double decimate,
                    double blur,
                    int threads,
                    bool debug,
                    bool refine_edges)
{
    ros::NodeHandle nh;
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("RPi_camera_v2/apriltag/pose", 1);

    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("RPi_camera_v2/image_raw", 1);

    sensor_msgs::ImagePtr msg;
    info = &pose_est_info;
    family_ = family;
    if (family_ == "tag36h11")
    {
        tf = tag36h11_create();
    }
    else if (family_ == "tag25h9")
    {
        tf = tag25h9_create();
    }
    else if (family_ == "tag16h5")
    {
        tf = tag16h5_create();
    }
    else if (family_ == "tagCircle21h7")
    {
        tf = tagCircle21h7_create();
    }
    else if (family_ == "tagCircle49h12")
    {
        tf = tagCircle49h12_create();
    }
    else if (family_ == "tagStandard41h12")
    {
        tf = tagStandard41h12_create();
    }
    else if (family_ == "tagStandard52h13")
    {
        tf = tagStandard52h13_create();
    }
    else if (family_ == "tagCustom48h12")
    {
        tf = tagCustom48h12_create();
    }
    else
    {
        std::cout << "Unrecognized tag family name: " << family_;
        return false;
    }

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = decimate;
    td->quad_sigma = blur;
    td->nthreads = threads;
    td->debug = debug;
    td->refine_edges = refine_edges;

    ros::Time::waitForValid();
    return true;
}

bool Detector::load_parameters(std::string filepath)
{
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open CamMatrix" << std::endl;
        exit(EXIT_FAILURE);
    }
    cv::Mat matrix;
    cv::Mat coeff;
    cv::Mat new_matrix; //matrix after undistort
    fs["camera_matrix"] >> matrix;
    fs["distortion_coefficients"] >> coeff;
    std::cout << "Matrix" << matrix << std::endl;
    std::cout << "Coeff" << coeff << std::endl;
    return 0;
}

void Detector::get_image(bool publish)
{
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    //   undistort(gray,un_gray,matrix,coeff,new_matrix);
    auto time_c = ros::Time::now();
    if (!frame.empty())
    {
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
        msg->header.stamp = time_c;
        if (publish)
            image_pub.publish(msg);
    }
}

void Detector::get_pose()
{
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    auto detections = apriltag_detector_detect(td, &im);
    auto time_c = ros::Time::now();

    if (zarray_size(detections) == 0)
    {
        pose_msg.header.stamp = time_c;
        pose_msg.pose = T_iden.toMsgsPose();
        pose_pub.publish(pose_msg);
    }
    for (int i = 0; i < zarray_size(detections); i++)
    {
        zarray_get(detections, i, &det);

        info->det = det;
        apriltag_pose_t pose;
        estimate_tag_pose(info, &pose);

        T_p_c = Tform(pose);
        pose_msg.header.stamp = time_c;
        pose_msg.pose = T_p_c.toMsgsPose();
        pose_pub.publish(pose_msg);
    }
    apriltag_detections_destroy(detections);
}