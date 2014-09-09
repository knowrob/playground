cmake_minimum_required(VERSION 2.8.3)
project(smach_tests)

find_package(catkin REQUIRED rospy smach smach_ros actionlib actionlib_msgs control_msgs trajectory_msgs json_prolog json_prolog_msgs knowrob_common)

catkin_package(
  CATKIN_DEPENDS rospy smach smach_ros actionlib actionlib_msgs control_msgs trajectory_msgs json_prolog json_prolog_msgs knowrob_common
)

