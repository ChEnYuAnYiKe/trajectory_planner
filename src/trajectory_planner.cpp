#include "trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner() {
	for (int i = 0; i < ROBOT_NUM; i++) {
		robots_.emplace_back(std::make_shared<Turtlebot>(i));
	}
	current_status_ = INIT;
	count = 0;
}

TrajectoryPlanner::~TrajectoryPlanner() {
}

void TrajectoryPlanner::init(ros::NodeHandle& nh) {
	nh.param("controller_freqency", controller_freqency_, 10.0);
	d_t_ = 1.0 / controller_freqency_;

	// 获取初始化位置
	for (int i = 0; i < ROBOT_NUM; i++) {
		nh.param("robot_" + std::to_string(i) + "_x", GoalPos(i, 0), 0.0);
		nh.param("robot_" + std::to_string(i) + "_y", GoalPos(i, 1), 0.0);
		nh.param("robot_" + std::to_string(i) + "_theta", GoalPos(i, 2), 0.0);
	}

	initRobot(nh);
	getAllGoalPos();
	runtime_timer_ = nh.createTimer(ros::Duration(d_t_), &TrajectoryPlanner::runCallback, this);

	ROS_INFO("Trajectory Planner Initialized");
}

void TrajectoryPlanner::initRobot(ros::NodeHandle& nh) {
	for (int i = 0; i < ROBOT_NUM; i++) {
		robots_[i]->init(nh);
	}
}

void TrajectoryPlanner::generateNewGoal() {
}

void TrajectoryPlanner::runCircle(std::shared_ptr<Turtlebot> robot) {
	// 机器人绕圆运动,半径为3m
	robot->cmd_vel.linear.x = 0.6;
	robot->cmd_vel.angular.z = 0.2;
	robot->twistOdom_.twist.twist.linear.x = robot->cmd_vel.linear.x;
	robot->twistOdom_.twist.twist.angular.z = robot->cmd_vel.angular.z;
}

void TrajectoryPlanner::getAllGoalPos() {
	for (int i = 0; i < ROBOT_NUM; i++) {
		robots_[i]->getGoalPos(GoalPos(i, 0), GoalPos(i, 1), GoalPos(i, 2));
	}
}

void TrajectoryPlanner::publishCmd() {
	for (int i = 0; i < ROBOT_NUM; i++) {
		robots_[i]->publisher();
	}
}

void TrajectoryPlanner::runCallback(const ros::TimerEvent& e) {
	switch (current_status_) {
	case INIT:
		for (int i = 0; i < ROBOT_NUM; i++) {
			robots_[i]->PIDController();
			// robots_[i]->publisher();
			if (robots_[i]->isGoalReached_) {
				count++;
			}
		}
		publishCmd();
		if (count == ROBOT_NUM) {
			current_status_ = PLANNING;
			ROS_INFO("All robots reached the initial position!");
		}else
			count = 0;

		break;

	case PLANNING:
		// generateNewGoal();
		for (int i = 0; i < ROBOT_NUM; i++) {
			runCircle(robots_[i]);
			// robots_[i]->publisher();
		}
		publishCmd();
		break;

	case END:
		break;
	}
}