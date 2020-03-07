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
    std::cout << "Destroying detector class" << std::endl;
    apriltag_detector_destroy(td_);
    if (family_ == "tag36h11")
    {
        tag36h11_destroy(tf_);
    }
    else if (family_ == "tag25h9")
    {
        tag25h9_destroy(tf_);
    }
    else if (family_ == "tag16h5")
    {
        tag16h5_destroy(tf_);
    }
    else if (family_ == "tagCircle21h7")
    {
        tagCircle21h7_destroy(tf_);
    }
    else if (family_ == "tagCircle49h12")
    {
        tagCircle49h12_destroy(tf_);
    }
    else if (family_ == "tagStandard41h12")
    {
        tagStandard41h12_destroy(tf_);
    }
    else if (family_ == "tagStandard52h13")
    {
        tagStandard52h13_destroy(tf_);
    }
    else if (family_ == "tagCustom48h12")
    {
        tagCustom48h12_destroy(tf_);
    }

    cap.release();
}

bool Detector::init(std::string family,
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

    family_ = family;
    if (family_ == "tag36h11")
    {
        tf_ = tag36h11_create();
    }
    else if (family_ == "tag25h9")
    {
        tf_ = tag25h9_create();
    }
    else if (family_ == "tag16h5")
    {
        tf_ = tag16h5_create();
    }
    else if (family_ == "tagCircle21h7")
    {
        tf_ = tagCircle21h7_create();
    }
    else if (family_ == "tagCircle49h12")
    {
        tf_ = tagCircle49h12_create();
    }
    else if (family_ == "tagStandard41h12")
    {
        tf_ = tagStandard41h12_create();
    }
    else if (family_ == "tagStandard52h13")
    {
        tf_ = tagStandard52h13_create();
    }
    else if (family_ == "tagCustom48h12")
    {
        tf_ = tagCustom48h12_create();
    }
    else
    {
        std::cout << "Unrecognized tag family name: " << family_;
        return false;
    }
    std::vector<float> T_cam;
    nh.getParam("/Transformation_camera_IMU", T_cam);
    T_c_v = Eigen::Matrix4f(T_cam.data()).transpose();
    // T_c_v.block<3, 1>(0, 3) = Eigen::Map<Eigen::Matrix<float, 3, 1>>(r_v_c_ptr);
    std::cout << T_c_v << "\n";

    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = decimate;
    td_->quad_sigma = blur;
    td_->nthreads = threads;
    td_->debug = debug;
    td_->refine_edges = refine_edges;

    ros::Time::waitForValid();
    return true;
}

bool Detector::load_parameters(std::string filepath)
{
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open CamMatrix" << std::endl;
        param_loaded = false;
        return param_loaded;
    }

    fs["camera_matrix"] >> matrix;
    fs["distortion_coefficients"] >> coeff;

    info_->tagsize = 0.0578;
    info_->fx = matrix.at<double>(0);
    info_->fy = matrix.at<double>(4);
    info_->cx = matrix.at<double>(2);
    info_->cy = matrix.at<double>(5);

    // Using printf here for pretty printing
    printf("Loaded camera matrix values:\n fx = %.04f fy = %.04f cx = %.04f cy = %.04f\n",
           info_->fx, info_->fy, info_->cx, info_->cy);
    printf("Loaded Distortion coefficients:\n%.04f %.04f %.04f %.04f %.04f\n",
           coeff.at<double>(0), coeff.at<double>(1), coeff.at<double>(2), coeff.at<double>(3), coeff.at<double>(4));
    param_loaded = true;
    return param_loaded;
}

void Detector::get_image(bool publish, bool apply_undistort)
{
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (param_loaded && apply_undistort)
        undistort(gray, un_gray, matrix, coeff, new_matrix);

    auto time_c = ros::Time::now();
    if (!frame.empty())
    {
        auto msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
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

    auto detections = apriltag_detector_detect(td_, &im);
    auto time_c = ros::Time::now();

    if (zarray_size(detections) == 0)
    {
        pose_msg.header.stamp = time_c;
        pose_msg.pose = T_iden.toMsgsPose();
        pose_pub.publish(pose_msg);
    }
    for (int i = 0; i < zarray_size(detections); i++)
    {
        zarray_get(detections, i, &det_);

        info_->det = det_;
        apriltag_pose_t pose;
        estimate_tag_pose(info_.get(), &pose);

        T_p_c = Tformf(pose);
        Tformf T_c_v;
        T_c_v << 0, 1, 0, 0,
            1, 0, 0, -0.05,
            0, 0, -1, 0.05,
            0, 0, 0, 1;

        pose_msg.header.stamp = time_c;
        pose_msg.pose = Tformf(T_c_v * T_p_c).toMsgsPose();
        pose_pub.publish(pose_msg);
    }
    apriltag_detections_destroy(detections);
}
