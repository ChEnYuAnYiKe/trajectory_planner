#include "tb_control.h"

#define ROBOT_NUM 4

Turtlebot::Turtlebot(int robotIDd) {
	robotID = robotIDd;
	initGoalReached_ = false;

	twistOdom_.twist.twist.linear.x = 0.0;
	twistOdom_.twist.twist.angular.z = 0.0;
	target_x_ = 0.0;
	target_y_ = 0.0;
}

Turtlebot::~Turtlebot() {
	ROS_INFO("Robot %d Node Terminated", robotID);
}

void Turtlebot::init(ros::NodeHandle& nh) {
	nh.param("p_window", p_window_, 0.2);
	nh.param("p_precision", p_precision_, 0.2);

	// max and min linear velocity
	nh.param("max_v", max_v_, 0.5);
	nh.param("min_v", min_v_, 0.0);
	nh.param("max_v_inc", max_v_inc_, 0.1); // maximum linear increase per time

	// max and min angular velocity
	nh.param("max_w", max_w_, 1.57);
	nh.param("min_w", min_w_, 0.0);
	nh.param("max_w_inc", max_w_inc_, 1.57); // maximum angular increase per time

	// the PID coefficients for linear velocity
	nh.param("k_v_p", k_v_p_, 1.0);
	nh.param("k_v_i", k_v_i_, 0.01);
	nh.param("k_v_d", k_v_d_, 0.1);

	// the PID coefficients for angular velocity
	nh.param("k_w_p", k_w_p_, 1.0);
	nh.param("k_w_i", k_w_i_, 0.01);
	nh.param("k_w_d", k_w_d_, 0.1);

	nh.param("controller_freqency", controller_freqency_, 10.0);
	d_t_ = 1.0 / controller_freqency_;

	e_v_ = i_v_ = 0.0;
	e_w_ = i_w_ = 0.0;

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("robot_" + std::to_string(robotID) + "_cmd_vel", 50);
	odom_sub_ = nh.subscribe<nav_msgs::Odometry>("robot_" + std::to_string(robotID) + "_odom", 50, &Turtlebot::odomSubCallback, this);
}

void Turtlebot::publisher() {
	vel_pub_.publish(cmd_vel);
}

void Turtlebot::odomSubCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	curOdom_ = *msg;
	x_ = curOdom_.pose.pose.position.x;
	y_ = curOdom_.pose.pose.position.y;
	theta_ = tf2::getYaw(curOdom_.pose.pose.orientation);
}

double Turtlebot::isGoalReached(double x_target, double y_target, double x, double y, double& dist) {
	dist = sqrt(pow(x_target - x, 2) + pow(y_target - y, 2));
	if (dist < p_precision_) {
		return true;
	}
	return false;
}