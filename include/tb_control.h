#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>

class Turtlebot {
public:
	Turtlebot(int robotIDd);
	~Turtlebot();
	void init(ros::NodeHandle& nh);
	void publisher();
	void odomSubCallback(const nav_msgs::Odometry::ConstPtr& msg);
	double isGoalReached(double x_target, double y_target, double x, double y, double& dist);
	void runOnce();

	int robotID;
	bool initGoalReached_;
	ros::Subscriber odom_sub_;
	ros::Publisher vel_pub_;

	nav_msgs::Odometry curOdom_, twistOdom_;
	geometry_msgs::Twist cmd_vel;

private:
	// cur odom's x, y, theta(yaw)
	double x_, y_, theta_;
	double p_window_;
	double p_precision_;
	double controller_freqency_, d_t_;
	double max_v_, min_v_, max_v_inc_;
	double max_w_, min_w_, max_w_inc_;
	double k_v_p_, k_v_i_, k_v_d_;
	double k_w_p_, k_w_i_, k_w_d_;
	// double k_theta_;

	double e_v_, e_w_;
	double i_v_, i_w_;
	double target_x_, target_y_;

	void regularizeAngle(double& angle);
	double LinearPIDController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d);
	double AngularPIDController(nav_msgs::Odometry& base_odometry, double theta_d, double theta);
};