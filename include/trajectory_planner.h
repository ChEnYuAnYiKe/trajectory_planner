#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "tb_control.h"

#define ROBOT_NUM 4

enum state {INIT, PLANNING, END};

class TrajectoryPlanner {
public:
	TrajectoryPlanner();
	~TrajectoryPlanner();
	void init(ros::NodeHandle& nh);

private:
	int current_status_;
	int count;
	double controller_freqency_, d_t_;
	ros::Timer runtime_timer_;
	Eigen::Matrix<double, ROBOT_NUM, 3> GoalPos;
	// std::vector<geometry_msgs::Twist> cmd_vels_;
	std::vector<std::shared_ptr<Turtlebot>> robots_;

	void initRobot(ros::NodeHandle& nh);
	void generateNewGoal();
	void runCircle(std::shared_ptr<Turtlebot> robot);
	void getAllGoalPos();
	void publishCmd();
	void runCallback(const ros::TimerEvent& e);
};