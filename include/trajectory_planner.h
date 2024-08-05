#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "tb_control.h"

class TrajectoryPlanner {
public:
	TrajectoryPlanner();
	~TrajectoryPlanner();
	void init(ros::NodeHandle& nh);
    void subscriber();
	void isGoalReached();
    double getGoalPositionDistance(double x_g, double y_g, double x, double y);
	void runCallback(const ros::TimerEvent& e);

private:
	ros::Timer runtime_timer_;
	Eigen::Matrix<double, 4, 3> initGoalPos;
    nav_msgs::Odometry cur_robot_0_odom, cur_robot_1_odom, cur_robot_2_odom, cur_robot_3_odom;
};