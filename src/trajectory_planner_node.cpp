#include "trajectory_planner.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "trajectory_planner");
	ros::NodeHandle nh("~");

	TrajectoryPlanner trajectory_planner;
    trajectory_planner.init(nh);

	ros::spin();

	return 0;
}