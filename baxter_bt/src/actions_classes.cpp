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

    void setPoint(std::vector<double> positions, double time=5.0) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = ros::Duration(time);
      goal_.trajectory.points.clear();
      goal_.trajectory.points.push_back(point);
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


class IKTrajectory : public Trajectory {

  protected:
    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
    baxter_core_msgs::SolvePositionIK srv_;
    geometry_msgs::PoseStamped pose_;

  public:
    IKTrajectory (std::string service_name="/ExternalTools/left/PositionKinematicsNode/IKService", 
                  std::string action_name="robot/limb/left/follow_joint_trajectory") 
      : service_client_(nh_.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name)), 
        Trajectory(action_name) {

      pose_.header.frame_id = "base";
      pose_.header.stamp = ros::Time::now();

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

        Trajectory::setPoint(point);
        return true;
      }
      else {
        ROS_ERROR("IK Service failed");
        return false;
      }
    }

};


class Gripper {

  protected:
    GripperClient client_;
    control_msgs::GripperCommandGoal goal_;

  public:
    Gripper (std::string server_name="/robot/end_effector/left_gripper/gripper_action") 
      : client_(server_name, true) {
      client_.waitForServer();
    }

    void setCommand(double position, double effort) {
      goal_.command.position = position;
      goal_.command.max_effort = effort;
      ROS_INFO("Sending goal to Gripper action server.");
      client_.sendGoal(goal_);
    }

    void stop() {
      client_.cancelAllGoals();
    }
    
    void wait(double timeout=5.0) {
      client_.waitForResult(ros::Duration(timeout));
    }

};

