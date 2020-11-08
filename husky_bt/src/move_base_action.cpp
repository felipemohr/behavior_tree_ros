#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_navigation_goals");

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))) 
    ROS_INFO("Waiting for the move_base action");

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = -2;
  goal.target_pose.pose.position.y = 2;
  goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Done!");
  else
    ROS_INFO("The base failed to move to the goal");

  return 0;
}
