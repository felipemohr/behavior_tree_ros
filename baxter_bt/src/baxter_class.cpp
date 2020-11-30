#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "actionlib/client/simple_action_client.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/EndpointState.h"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


class BaxterArm {

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    TrajectoryClient traj_client_;
    GripperClient gripper_client_;
    ros::ServiceClient ik_service_client_;
    baxter_core_msgs::SolvePositionIK ik_srv_;
    control_msgs::FollowJointTrajectoryGoal joints_goal_;
    geometry_msgs::PoseStamped pose_goal_;
    geometry_msgs::Pose gripper_pose_;
    control_msgs::GripperCommandGoal gripper_cmd_;


    void gripperPoseCallback(const baxter_core_msgs::EndpointState::ConstPtr &current_gripper_pose) {
      gripper_pose_ = current_gripper_pose->pose;
    }

  
  public:
    BaxterArm ( std::string limb = "left" ) 
      : pose_sub_(nh_.subscribe("/robot/limb/"+limb+"/endpoint_state", 10, &BaxterArm::gripperPoseCallback, this)),
        traj_client_("robot/limb/"+limb+"/follow_joint_trajectory", true), 
        gripper_client_("/robot/end_effector/"+limb+"_gripper/gripper_action", true),
        ik_service_client_(nh_.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/"+limb+"/PositionKinematicsNode/IKService")) {

      if ((limb != "left") && (limb != "right"))
        ROS_WARN("By default, 'limb' parameter in BaxterArm constructor must be 'left' or 'right'");

      traj_client_.waitForServer();
      gripper_client_.waitForServer();

      joints_goal_.goal_time_tolerance = ros::Duration(0.1);

      joints_goal_.trajectory.joint_names.push_back(limb+"_s0");
      joints_goal_.trajectory.joint_names.push_back(limb+"_s1");
      joints_goal_.trajectory.joint_names.push_back(limb+"_e0");
      joints_goal_.trajectory.joint_names.push_back(limb+"_e1");
      joints_goal_.trajectory.joint_names.push_back(limb+"_w0");
      joints_goal_.trajectory.joint_names.push_back(limb+"_w1");
      joints_goal_.trajectory.joint_names.push_back(limb+"_w2");

      pose_goal_.header.frame_id = "base";
      pose_goal_.header.stamp = ros::Time::now();

    }

    void setJoints(std::vector<double> positions, double time=5.0) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.time_from_start = ros::Duration(time);
      joints_goal_.trajectory.points.clear();
      joints_goal_.trajectory.points.push_back(point);
    }

    bool setPose(geometry_msgs::Pose goal) {
      pose_goal_.pose = goal;
      ik_srv_.request.pose_stamp.push_back(pose_goal_);
      if (ik_service_client_.call(ik_srv_)) {
        std::vector<double> point;
        sensor_msgs::JointState js;
        
        js = ik_srv_.response.joints[0];
        for (int p=0; p<js.position.size(); p++) 
          point.push_back(js.position[p]);
        
        ROS_INFO("Joints positions calculated");

        setJoints(point);
        return true;
      }
      else {
        ROS_ERROR("IK Service failed");
        return false;
      }
    }

    bool offsetPose(geometry_msgs::Pose offset) {
      getGripperPose();
      geometry_msgs::Pose new_goal;
      new_goal.orientation = gripper_pose_.orientation;
      new_goal.position.x = gripper_pose_.position.x + offset.position.x;
      new_goal.position.y = gripper_pose_.position.y + offset.position.y;
      new_goal.position.z = gripper_pose_.position.z + offset.position.z;
      return setPose(new_goal);
    }

    void setGripper(double position, double effort) {
      gripper_cmd_.command.position = position;
      gripper_cmd_.command.max_effort = effort;
      ROS_INFO("Sending goal to Gripper action server.");
      gripper_client_.sendGoal(gripper_cmd_);
    }

    void moveJoints() {
      joints_goal_.trajectory.header.stamp = ros::Time::now();
      ROS_INFO("Sending goal to Trajectory action server.");
      traj_client_.sendGoal(joints_goal_);
    }

    geometry_msgs::Pose getGripperPose() {
      ros::spinOnce();
      return gripper_pose_;
    }

    void stop() {
      traj_client_.cancelAllGoals();
    }

    void stopGripper() {
      gripper_client_.cancelAllGoals();
    }

    void wait(double timeout=15.0) {
      traj_client_.waitForResult(ros::Duration(timeout));
    }

    void waitGripper(double timeout=5.0) {
      gripper_client_.waitForResult(ros::Duration(timeout));
    }

};
