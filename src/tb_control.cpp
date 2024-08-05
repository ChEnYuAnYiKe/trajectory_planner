#include "tb_control.h"

Turtlebot::Turtlebot(int robotID): robotID_(robotID) {
	isGoalReached_ = false;

	twistOdom_.twist.twist.linear.x = 0.0;
	twistOdom_.twist.twist.angular.z = 0.0;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
	x_target_ = 0.0;
	y_target_ = 0.0;
}

Turtlebot::~Turtlebot() {
}

void Turtlebot::init(ros::NodeHandle& nh) {
	nh.param("p_precision", p_precision_, 0.2);
    nh.param("o_precision", o_precision_, 0.5);

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

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("robot_" + std::to_string(robotID_) + "_cmd_vel", 50);
	odom_sub_ = nh.subscribe<nav_msgs::Odometry>("robot_" + std::to_string(robotID_) + "_odom", 50, &Turtlebot::odomSubCallback, this);
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

void Turtlebot::getGoalPos(double x_target, double y_target, double theta_target) {
    x_target_ = x_target;
    y_target_ = y_target;
    theta_target_ = theta_target;
    isGoalReached_ = false;
}

double Turtlebot::getGoalPositionDistance() {
	return sqrt(pow(x_target_ - x_, 2) + pow(x_target_ - x_, 2));
}

void Turtlebot::PIDController() {
	double x_d, y_d;
	double e_theta, theta_d;

	// 期望的位置和偏差的角度
    theta_d = atan2((y_target_ - y_), (x_target_ - x_));
    regularizeAngle(theta_d);
    e_theta = theta_d - theta_;
    regularizeAngle(e_theta);

    if (getGoalPositionDistance() < p_precision_)
    {
        e_theta = theta_target_ - theta_;
        regularizeAngle(e_theta);

        if (std::fabs(e_theta) < o_precision_)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            isGoalReached_ = true;
            // ROS_ERROR("STOP");
            // this->isGoalReached();
        }
        // orientation not reached
        else
        {
            cmd_vel.linear.x = 0.0;
            // cmd_vel.angular.z = 0.0;
            cmd_vel.angular.z = AngularPIDController(twistOdom_, theta_target_, theta_);
			// ROS_WARN("adjust angle!!");
		}
	}
    else if (std::fabs(e_theta) > 0.5)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = AngularPIDController(twistOdom_, theta_d, theta_);
        // ROS_INFO("angle big");
    }
    else
    {
        cmd_vel.linear.x = LinearPIDController(twistOdom_, x_target_, y_target_);
        cmd_vel.angular.z = AngularPIDController(twistOdom_, theta_d, theta_);
    }

    twistOdom_.twist.twist.linear.x = cmd_vel.linear.x;
    twistOdom_.twist.twist.angular.z = cmd_vel.angular.z;
    // ROS_WARN("[debug]:angular.z=%f",cmd_vel.angular.z);
    // vel_pub_.publish(cmd_vel);

    // ROS_WARN("vel_x:%f,ang_z:%f", cmd_vel.linear.x, cmd_vel.angular.z);
}    

// 将角度转换为[-pi,pi]
void Turtlebot::regularizeAngle(double &angle)
{
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

double Turtlebot::LinearPIDController(nav_msgs::Odometry &base_odometry, double b_x_d, double b_y_d)
{
    double v = base_odometry.twist.twist.linear.x;
    // ROS_WARN("x:%.2lf,y:%.2lf",base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);

    // 距离越大，期望的速度越大
    double v_d = std::hypot(b_x_d, b_y_d) / d_t_;
    if (std::fabs(v_d) > max_v_)
        v_d = std::copysign(max_v_, v_d);
    // ROS_WARN("v_d: %.2f", v_d);

    double e_v = v_d - v;
    i_v_ += e_v * d_t_;
    double d_v = (e_v - e_v_) / d_t_;
    e_v_ = e_v;

    // v_inc <---> v increase
    double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;

    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);

    double v_cmd = v + v_inc;
    if (std::fabs(v_cmd) > max_v_)
        v_cmd = std::copysign(max_v_, v_cmd);
    else if (std::fabs(v_cmd) < min_v_)
        v_cmd = std::copysign(min_v_, v_cmd);

    // ROS_INFO("v_d: %.2lf, e_v: %.2lf, i_v: %.2lf, d_v: %.2lf, v_cmd: %.2lf", v_d, e_v, i_v_, d_v, v_cmd);
    // ROS_INFO("v: %.2lf, v_inc: %.2lf, v_cmd: %.2lf", v, v_inc, v_cmd);

    // twistOdom_.twist.twist.linear.x = v_cmd;
    return v_cmd;
}

double Turtlebot::AngularPIDController(nav_msgs::Odometry &base_odometry, double theta_d, double theta)
{
    double e_theta = theta_d - theta;
    /// ROS_WARN("theta_d: %.2lf,theta: %.2lf",theta_d,theta);
    regularizeAngle(e_theta);

    double w_d = e_theta / d_t_;
    if (std::fabs(w_d) > max_w_)
        w_d = std::copysign(max_w_, w_d);
    // ROS_WARN("w_d: %.2f", w_d);

    double w = base_odometry.twist.twist.angular.z;
    double e_w = w_d - w;
    i_w_ += e_w * d_t_;
    double d_w = (e_w - e_w_) / d_t_;
    e_w_ = e_w;

    double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

    if (std::fabs(w_inc) > max_w_inc_)
        w_inc = std::copysign(max_w_inc_, w_inc);

    double w_cmd = w + w_inc;
    if (std::fabs(w_cmd) > max_w_)
        w_cmd = std::copysign(max_w_, w_cmd);
    else if (std::fabs(w_cmd) < min_w_)
        w_cmd = std::copysign(min_w_, w_cmd);

    // ROS_INFO("w_d: %.2lf, e_w: %.2lf, i_w: %.2lf, d_w: %.2lf, w_cmd: %.2lf", w_d, e_w, i_w_, d_w, w_cmd);
    // ROS_INFO("w_d: %.2lf, w: %.2lf,w_inc: %.2lf,  w_cmd: %.2lf", w_d, w,w_inc, w_cmd);
    w_cmd = k_w_p_ * w_d;
    // twistOdom_.twist.twist.angular.z = w_cmd;
    return w_cmd;
}