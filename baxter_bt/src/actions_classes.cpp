#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "actionlib/client/simple_action_client.h"
#include "baxter_core_msgs/SolvePositionIK.h"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


class Trajectory {

  protected:
    TrajectoryClient client_;
    control_msgs::FollowJointTrajectoryGoal goal_;
    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
    baxter_core_msgs::SolvePositionIK srv_;
    geometry_msgs::PoseStamped pose_;

  
  public:
    Trajectory (std::string server_name="robot/limb/left/follow_joint_trajectory",
                std::string service_name="/ExternalTools/left/PositionKinematicsNode/IKService",
                std::string action_name="robot/limb/left/follow_joint_trajectory") 
      : client_(server_name, true), 
      service_client_(nh_.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name)) {

      client_.waitForServer();

      goal_.goal_time_tolerance = ros::Duration(0.1);

      goal_.trajectory.joint_names.push_back("left_s0");
      goal_.trajectory.joint_names.push_back("left_s1");
      goal_.trajectory.joint_names.push_back("left_e0");
      goal_.trajectory.joint_names.push_back("left_e1");
      goal_.trajectory.joint_names.push_back("left_w0");
      goal_.trajectory.joint_names.push_back("left_w1");
      goal_.trajectory.joint_names.push_back("left_w2");

      pose_.header.frame_id = "base";
      pose_.header.stamp = ros::Time::now();

    }

    void setPoint(std::vector<double> positions, double time=5.0) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = ros::Duration(time);
      goal_.trajectory.points.clear();
      goal_.trajectory.points.push_back(point);
    }

    bool setPose (geometry_msgs::Pose goal) {
      pose_.pose = goal;
      srv_.request.pose_stamp.push_back(pose_);
      if (service_client_.call(srv_)) {
        std::vector<double> point;
        sensor_msgs::JointState js;
        
        js = srv_.response.joints[0];
        for (int p=0; p<js.position.size(); p++) 
          point.push_back(js.position[p]);
        
        ROS_INFO("Joints positions calculated");

        setPoint(point);
        return true;
      }
      else {
        ROS_ERROR("IK Service failed");
        return false;
      }
    }

    void start() {
      goal_.trajectory.header.stamp = ros::Time::now();
      ROS_INFO("Sending goal to Trajectory action server.");
      client_.sendGoal(goal_);
    }

    void stop() {
      client_.cancelAllGoals();
    }

    void wait(double timeout=15.0) {
      client_.waitForResult(ros::Duration(timeout));
    }

};




class Gripper {

  protected:
    GripperClient gripper_client_;
    control_msgs::GripperCommandGoal gripper_cmd_;

  public:
    Gripper (std::string server_name="/robot/end_effector/left_gripper/gripper_action") 
      : gripper_client_(server_name, true) {
      gripper_client_.waitForServer();
    }

    void setCommand(double position, double effort) {
      gripper_cmd_.command.position = position;
      gripper_cmd_.command.max_effort = effort;
      ROS_INFO("Sending goal to Gripper action server.");
      gripper_client_.sendGoal(gripper_cmd_);
    }

    void stop() {
      gripper_client_.cancelAllGoals();
    }
    
    void wait(double timeout=5.0) {
      gripper_client_.waitForResult(ros::Duration(timeout));
    }

};

