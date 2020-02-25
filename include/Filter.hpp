//This is a constant velocity extend kalman filter implementation

#include "common.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class Filter
{
public:
	static int constexpr N_st = 6; // Dimension of state
	static int constexpr N_ob = 6; // Dimension of observation
	//ctor
	Filter(const MatrixXf<N_st> &P0, const MatrixXf<N_st> &Q0, const MatrixXf<N_ob> &R0);

	bool ini = false;
	bool loss = false;
	bool pub_flag = false;

	const static int M = 6;
	unsigned int F_count = 0;

	std::vector<std::pair<float, Eigen::Vector3f>> position_data;

	MatrixXf<N_st> F, P, Q, P_pre, P_temp; //state transformtion matrix and covariance matrix and process noise
	Matrix<float, N_ob, N_st> H;		   //observation matrix
	Matrix<float, N_st, N_ob> K;		   //gain
	MatrixXf<M> R;						   // covariance of observation
	VectorXf<M> Z;						   // observation
	VectorXf<N_st> X, X_pre, X_temp;

	void receivedata(const geometry_msgs::PoseStamped::ConstPtr &msg);

	void publishpose(const ros::Publisher &pub);

	void predict();

	void compute_proc_Jacobian();
	
	void compute_obsv_Jacobian();

	void update(Eigen::Matrix<float, N_ob, 1> &observe);
};

//Publish message if predict and update has achieved a new message the flag will be 1
