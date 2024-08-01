#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class TrajectoryPlanner {
public:
	TrajectoryPlanner();
	~TrajectoryPlanner();
	void init(ros::NodeHandle& nh);
    void subscriber();
	void isGoalReached();
    double getGoalPositionDistance(double x_g, double y_g, double x, double y);
    void subOdom_robot_0(const nav_msgs::OdometryPtr &msg);
    void subOdom_robot_1(const nav_msgs::OdometryPtr &msg);
    void subOdom_robot_2(const nav_msgs::OdometryPtr &msg);
    void subOdom_robot_3(const nav_msgs::OdometryPtr &msg);
    void runCallback(const ros::TimerEvent &e);

private:
	ros::Subscriber robot_0_odom_sub_, robot_1_odom_sub_, robot_2_odom_sub_, robot_3_odom_sub_;
	ros::Publisher robot_0_cmd_vel_pub_, robot_1_cmd_vel_pub_, robot_2_cmd_vel_pub_, robot_3_cmd_vel_pub_;
    ros::Timer runtime_timer_;

	Eigen::Matrix<double, 4, 3> robot_initGoalPos;
    nav_msgs::Odometry cur_robot_0_odom, cur_robot_1_odom, cur_robot_2_odom, cur_robot_3_odom;
	bool robot_0_initGoalReached, robot_1_initGoalReached, robot_2_initGoalReached, robot_3_initGoalReached;
	double p_precision_;
    
};