#include "actions_classes.cpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

namespace BT {
  template <> std::vector<double> convertFromString(StringView key) {
    auto input = BT::splitString(key, ' ');

    std::vector<double> point;
    for(auto j:input)
      point.push_back(convertFromString<double>(j));

    return point;
  }


  template <> geometry_msgs::Pose convertFromString(StringView key) {
    auto input = BT::splitString(key, ' ');

    geometry_msgs::Pose pose;
    if(input.size() == 3) {
      pose.position.x = convertFromString<double>(input[0]);
      pose.position.y = convertFromString<double>(input[1]);
      pose.position.z = convertFromString<double>(input[2]);
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      return pose;
    }
    else if(input.size() == 7) {
      pose.position.x = convertFromString<double>(input[0]);
      pose.position.y = convertFromString<double>(input[1]);
      pose.position.z = convertFromString<double>(input[2]);
      pose.orientation.x = convertFromString<double>(input[3]);
      pose.orientation.y = convertFromString<double>(input[4]);
      pose.orientation.z = convertFromString<double>(input[5]);
      pose.orientation.w = convertFromString<double>(input[6]);
      return pose;
    }
    else {
      throw RuntimeError("Invalid Input.");
    }
  }


  template <> control_msgs::GripperCommandGoal convertFromString(StringView key) {
    auto input = BT::splitString(key, ' ');

    control_msgs::GripperCommandGoal cmd;
    if(input.size() == 1) {
      cmd.command.position = convertFromString<double>(input[0]); 
      cmd.command.max_effort = 50.0;
      return cmd;
    }
    else if(input.size() == 2) {
      cmd.command.position = convertFromString<double>(input[0]); 
      cmd.command.max_effort = convertFromString<double>(input[1]);
      return cmd;
    }
    else
      throw RuntimeError("Invalid Input.");
  }

}


class TrajectoryAction : public BT::AsyncActionNode, public Trajectory {

  public:
    TrajectoryAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), Trajectory("robot/limb/left/follow_joint_trajectory", 
                   "/ExternalTools/left/PositionKinematicsNode/IKService", 
                   "robot/limb/left/follow_joint_trajectory") {
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort< std::vector<double> >("commands")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      halt_requested_ = true;
    }

  private:
    bool halt_requested_;

};

BT::NodeStatus TrajectoryAction::tick() {

  ROS_INFO("Waiting for Trajectory action server to start");
  if(!client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Trajectory server");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<double> point;
  if(!getInput< std::vector<double> >("commands", point))
    throw BT::RuntimeError("Missing required input [commands]");

  Trajectory::setPoint(point, 7.0);
  Trajectory::start();

  ROS_INFO("Trajectory Action Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    Trajectory::stop();
    ROS_ERROR("Trajectory Action aborted");
    return BT::NodeStatus::FAILURE; 
  }

  ROS_INFO("Trajectory Action finished. ");
  return BT::NodeStatus::SUCCESS;

}


class IKTrajectoryAction : public BT::AsyncActionNode, public Trajectory {

  public:
    IKTrajectoryAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), 
        Trajectory("robot/limb/left/follow_joint_trajectory", 
                   "/ExternalTools/left/PositionKinematicsNode/IKService", 
                   "robot/limb/left/follow_joint_trajectory") {
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort<geometry_msgs::Pose>("pose")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      halt_requested_ = true;
    }

  private:
    bool halt_requested_;

};

BT::NodeStatus IKTrajectoryAction::tick() {
  ROS_INFO("Waiting for Trajectory action server to start");
  if(!client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Trajectory server");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::Pose pose;
  if(!getInput<geometry_msgs::Pose>("pose", pose))
    throw BT::RuntimeError("Missing required input [pose]");

  setPose(pose);
  start();

  ROS_INFO("Trajectory Action IK Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    stop();
    ROS_ERROR("Trajectory Action IK aborted");
    return BT::NodeStatus::FAILURE;
  }
  
  ROS_INFO("Trajectory Action IK finished. ");
  return BT::NodeStatus::SUCCESS;
  
}


class GripperAction : public BT::AsyncActionNode, public Gripper {

  public:
    GripperAction(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::AsyncActionNode(name, config), Gripper("/robot/end_effector/left_gripper/gripper_action") {
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort<double>("position")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      halt_requested_ = true;
    }

  private:
    bool halt_requested_;

};

BT::NodeStatus GripperAction::tick() {

  ROS_INFO("Waiting for Gripper action server to start");
  if(!gripper_client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Gripper Action server");
    return BT::NodeStatus::FAILURE;
  }

  control_msgs::GripperCommandGoal cmd;
  if(!getInput<control_msgs::GripperCommandGoal>("position", cmd))
    throw BT::RuntimeError("Missing required input [position]");

  Gripper::setCommand(cmd.command.position, cmd.command.max_effort);

  ROS_INFO("Gripper Action Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !gripper_client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    Gripper::stop();
    ROS_ERROR("Gripper Action aborted");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Gripper Action finished. ");
  return BT::NodeStatus::SUCCESS;
}
