#! /bin/bash
# clockwise
rostopic pub /robot_3/cmd_vel -r 50 geometry_msgs/Twist "linear:
  x: 0.6
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.2"
