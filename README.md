# tx2_action_server
Action server for performing exploration in simulated environments.

Is used with [m-explore](https://github.com/cnndepth/m-explore) exploration node and [ThetaStarROS](https://github.com/haiot4105/ThetaStarROS) path planning node.

## Requirements
* ROS version at least Kinetic
* CMake version at least 2.8.3
* Python 2.7
* Python packages:
  * Actionlib
  * Transformations
  
## Usage

### With external navigation (e.g. from keyboard)

rosrun tx2_action_server tx2_action_server_external_driver.py

### With embedded navigation

TODO

## Input and output

### Subscribed topics

* odom (`nav_msgs::Odometry`) - robot's position
* tf - transform from map frame to odom_frame

### Used servers
* move_base (`actionlib.SimpleActionServer`)

### Published topics

* exploration_goal (`geometry_msgs::PoseStamped`) - position of goal
* task (`std_msgs::Float32MultiArray`) - coordinates for path planner
