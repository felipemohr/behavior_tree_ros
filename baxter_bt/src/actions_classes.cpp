#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "actionlib/client/simple_action_client.h"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class Trajectory {

  protected:
    TrajectoryClient client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

  
  public:
    Trajectory (std::string server_name="robot/limb/left/follow_joint_trajectory") 
      : client_(server_name, true) {

        client_.waitForServer();

        goal_.goal_time_tolerance = ros::Duration(0.1);

        goal_.trajectory.joint_names.push_back("left_s0");
        goal_.trajectory.joint_names.push_back("left_s1");
        goal_.trajectory.joint_names.push_back("left_e0");
        goal_.trajectory.joint_names.push_back("left_e1");
        goal_.trajectory.joint_names.push_back("left_w0");
        goal_.trajectory.joint_names.push_back("left_w1");
        goal_.trajectory.joint_names.push_back("left_w2");

    }

    void setPoint(std::vector<double> positions, double time) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = ros::Duration(time);
      goal_.trajectory.points.clear();
      goal_.trajectory.points.push_back(point);
    }

    void start() {
      goal_.trajectory.header.stamp = ros::Time::now();
      ROS_INFO("Sending goal to action server.");
      client_.sendGoal(goal_);
    }

    void stop() {
      client_.cancelAllGoals();
    }

    void wait(double timeout=15.0) {
      client_.waitForResult(ros::Duration(timeout));
    }

};

