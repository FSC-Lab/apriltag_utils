#include "../include/Filter.hpp"

Filter::Filter(const MatrixXf<N_st> &P0, const MatrixXf<N_st> &Q, const MatrixXf<N_ob> &R) : P(P0), Q(Q), R(R)
{
    X.block<3, 1>(0, 0) = Eigen::MatrixXf::Zero(3, 1);
    X.block<3, 1>(3, 0) = Eigen::MatrixXf::Zero(3, 1);

    X_pre = X;
    P_pre = P;

    K = Eigen::MatrixXf::Zero(N_ob, N_st);
}

void Filter::receivedata(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Matrix<float, N_ob, 1> measurement;

    Vecf r_p_c(msg->pose.position);
    measurement.block<3, 1>(0, 0) = r_p_c;

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
        // set_initial_state(r_p_c);
        this->ini = 1;
        pub_flag = 1;
    }
    else if (this->ini == 1 && r_p_c.norm() != 0)
    {
        position_data.push_back(std::make_pair(static_cast<float>(msg->header.stamp.toSec()), r_p_c)); //record data to calculate velocity

        if (sizeof(position_data) > 10)
        {
            Eigen::Vector3f Velocity;
            float dt = position_data.back().first - position_data.end()[-2].first;
            if (dt > 0)
                Velocity = (position_data.back().second - position_data.end()[-2].second) / dt;

            measurement.block<3, 1>(3, 0) = Velocity;
        }
        std::cout << "Measurement" << measurement << std::endl;

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

        Vecf r_c_v(0.02, 0.05, -0.02);
        pose_msg.pose.pose.position = Vecf(X.block<3, 1>(0, 0) + r_c_v).toMsgsPoint();
        pose_msg.pose.covariance[0] = P(1, 1);
        pose_msg.pose.covariance[7] = P(0, 0);
        pose_msg.pose.covariance[15] = P(2, 2);
        pose_msg.pose.covariance[1] = 0;
        //rotation is comment not used in single drone
        /*Eigen::Matrix3f relative_rotation = relative_T.block<3,3>(0,0);
          Eigen::Quaternion<double> q_relative(relative_rotation);
          pose_msg.pose.pose.orientation.x = q_relative.x();
          pose_msg.pose.pose.orientation.y = q_relative.y();
          pose_msg.pose.pose.orientation.z = q_relative.z();
          pose_msg.pose.pose.orientation.w = q_relative.w();*/
        pose_msg.twist.twist.linear = Vecf(X.block<3, 1>(3, 5)).toMsgsVector3();

        if (loss == 1)
            pose_msg.pose.covariance[1] = 1;
        pub.publish(pose_msg);
        pub_flag = 0; //after publishing it will be reset to zero
    }
}

void Filter::compute_proc_Jacobian()
{
    float dt = 1.0f / 100;
    F << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
}

void Filter::predict()
{
    compute_proc_Jacobian();

    X_pre = F * X;                     // onestep predict state
    P_pre = F * P * F.transpose() + Q; //covariance predict

    //Record value
    X = X_pre;
    P = P_pre;
    pub_flag = 1;

    std::cout << "Predicted state" << X.transpose() << std::endl;
}

void Filter::compute_obsv_Jacobian()
{
    H.setIdentity();
}

void Filter::update(Eigen::Matrix<float, N_ob, 1> &observe)
{
    compute_obsv_Jacobian();
    Z = observe;
    K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
    X = X_pre + K * (Z - H * X_pre);
    P = (Eigen::MatrixXf::Identity(N_ob, N_ob) - K * H) * P_pre;
    std::cout << "Updated state" << X.transpose() << std::endl;
}
