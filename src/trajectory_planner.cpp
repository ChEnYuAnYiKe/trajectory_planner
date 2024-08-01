#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner() {
    robot_0_initGoalReached = false;
    robot_1_initGoalReached = false;
    robot_2_initGoalReached = false;
    robot_3_initGoalReached = false;
	ROS_INFO("Trajectory Planner Node Started");
}

TrajectoryPlanner::~TrajectoryPlanner() {
	ROS_INFO("Trajectory Planner Node Terminated");
}

void TrajectoryPlanner::init(ros::NodeHandle& nh) {
    nh.param<double>("p_precision", p_precision_, 0.1);

    
    runtime_timer_ = nh.createTimer(ros::Duration(0.1), &TrajectoryPlanner::runCallback, this);
    subscriber();
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "trajectory_planner");
	ros::NodeHandle nh("~");

	TrajectoryPlanner trajectory_planner;
    trajectory_planner.init(nh);

	ros::spin();

	return 0;
}