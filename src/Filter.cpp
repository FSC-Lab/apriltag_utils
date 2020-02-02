#include "../include/Filter.hpp"

Filter::Filter(const MatrixXd<N_st> &P0, const MatrixXd<N_st> &Q, const MatrixXd<N_ob> &R) : P(P0), Q(Q), R(R)
{
    X.block<3, 1>(0, 0) = Eigen::MatrixXd::Zero(3, 1);
    X.block<3, 1>(3, 0) = Eigen::MatrixXd::Zero(3, 1);

    X_pre = X;
    P_pre = P;

    K = Eigen::MatrixXd::Zero(N_ob, N_st);
}

void Filter::receivedata(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Matrix<double, N_ob, 1> measurement;

    

    measurement(0) = msg->pose.position.x;
    measurement(1) = msg->pose.position.y;
    measurement(2) = msg->pose.position.z; //compensate some offset
    Eigen::Vector3d p_measure = measurement.block<3, 1>(0, 0);

    // std::cout << measurement << std::endl; detect loss
    if (F_count > 10) //losr for a long time 10times
        loss = true;

    if (measurement.norm() == 0)
    { //detect lost
        F_count++;
        return;
    }

    //detect
    else if (this->ini == 0)
    { // if not initilization
        // set_initial_state(p_measure);
        this->ini = 1;
        pub_flag = 1;
    }
    else if (this->ini == 1 && p_measure.norm() != 0)
    {
        position_data.push_back(std::make_pair(msg->header.stamp.toSec(), p_measure)); //record data to calculate velocity

        if (sizeof(position_data) > 10)
        {
            Eigen::Vector3d Velocity;
            double dt = position_data.back().first - position_data[position_data.size() - 2].first;
            if (dt > 0)
                Velocity = (position_data.back().second - position_data[position_data.size() - 2].second) / dt;
            //std::cout << "time is " << dt << std::endl<<"v====isss   " << Velocity << std::endl << "posi is  " << position_data.back().second
            //   << "posi old is  " << position_data[position_data.size()-2].second << std::endl;
            measurement.block<3, 1>(3, 0) = Velocity;
        }

        loss = false;
        this->update(measurement);
        pub_flag = 1;
        F_count = 0;
    }
}

void Filter::publishpose(const ros::Publisher &pub)
{
    if (pub_flag == 1)
    {
        nav_msgs::Odometry pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "filter_apriltag_tag";
        // There need to be compensate with attitude of UAV , if have yaw angle it will cause more error;
        // now the temp compensation is 0.03 -0.05 -0.02 should be multiple by a rotation matrix

        Vec r_c_v(0.02, 0.05, -0.02);
        pose_msg.pose.pose.position = Vec(X.block<3, 1>(0, 0) + r_c_v).toMsgsPoint();
        pose_msg.pose.covariance[0] = P(1, 1);
        pose_msg.pose.covariance[7] = P(0, 0);
        pose_msg.pose.covariance[15] = P(2, 2);
        pose_msg.pose.covariance[1] = 0;
        //rotation is comment not used in single drone
        /*Eigen::Matrix3d relative_rotation = relative_T.block<3,3>(0,0);
          Eigen::Quaternion<double> q_relative(relative_rotation);
          pose_msg.pose.pose.orientation.x = q_relative.x();
          pose_msg.pose.pose.orientation.y = q_relative.y();
          pose_msg.pose.pose.orientation.z = q_relative.z();
          pose_msg.pose.pose.orientation.w = q_relative.w();*/
        pose_msg.twist.twist.linear.x = X(4);
        pose_msg.twist.twist.linear.y = X(3);
        pose_msg.twist.twist.linear.z = -X(5);

        if (loss == 1)
            pose_msg.pose.covariance[1] = 1;
        pub.publish(pose_msg);
        pub_flag = 0; //after publishing it will be reset to zero
    }
}

void Filter::predict()
{
    X_pre = F * X;                     // onestep predict state
    P_pre = F * P * F.transpose() + Q; //covariance predict

    //Record value
    X = X_pre;
    P = P_pre;
    pub_flag = 1;

    //   std::cout << X  << std::endl << F  << std::endl << P  << std::endl;
}

void Filter::update(Eigen::Matrix<double, N_ob, 1> &observe)
{
    Z = observe;
    K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
    X = X_pre + K * (Z - H * X_pre);
    P = (Eigen::MatrixXd::Identity(N_ob, N_ob) - K * H) * P_pre;
}

void Filter::matToConsole()
{
    std::cout << P << std::endl
              << Q << std::endl
              << R << std::endl;
}