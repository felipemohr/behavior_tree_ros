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


  template <> geometry_msgs::Point convertFromString(StringView key) {
    auto input = BT::splitString(key, ' ');

    geometry_msgs::Point position;
    if(input.size() == 3) {
      position.x = convertFromString<double>(input[0]);
      position.y = convertFromString<double>(input[1]);
      position.z = convertFromString<double>(input[2]);
      return position;
    }
    else {
      throw RuntimeError("Invalid Input.");
    }
  }


  template <> geometry_msgs::Quaternion convertFromString(StringView key) {
    auto input = BT::splitString(key, ' ');

    geometry_msgs::Quaternion orientation;
    if(input.size() == 4) {
      orientation.x = convertFromString<double>(input[0]);
      orientation.y = convertFromString<double>(input[1]);
      orientation.z = convertFromString<double>(input[2]);
      orientation.w = convertFromString<double>(input[3]);
      return orientation;
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


class MoveJointsActionNode : public BT::AsyncActionNode, public BaxterArm {

  public:
    MoveJointsActionNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), 
        BaxterArm("left") {
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

BT::NodeStatus MoveJointsActionNode::tick() {

  ROS_INFO("Waiting for Trajectory action server to start");
  if(!traj_client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Trajectory server");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<double> point;
  if(!getInput< std::vector<double> >("commands", point))
    throw BT::RuntimeError("Missing required input [commands]");

  BaxterArm::setJoints(point, 7.0);
  BaxterArm::moveJoints();

  ROS_INFO("Trajectory Action Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !traj_client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    BaxterArm::stop();
    ROS_ERROR("Trajectory Action aborted");
    return BT::NodeStatus::FAILURE; 
  }

  ROS_INFO("Trajectory Action finished. ");
  return BT::NodeStatus::SUCCESS;

}


class MoveToPoseActionNode : public BT::AsyncActionNode, public BaxterArm {

  public:
    MoveToPoseActionNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), 
        BaxterArm("left") {
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort<geometry_msgs::Point>("position"),
              BT::InputPort<geometry_msgs::Quaternion>("orientation")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      halt_requested_ = true;
    }

  private:
    bool halt_requested_;

};

BT::NodeStatus MoveToPoseActionNode::tick() {
  ROS_INFO("Waiting for Trajectory action server to start");
  if(!traj_client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Trajectory server");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::Pose pose;
  if(!getInput<geometry_msgs::Point>("position", pose.position))
    throw BT::RuntimeError("Missing required input [position]");
  if(!getInput<geometry_msgs::Quaternion>("orientation", pose.orientation))
    throw BT::RuntimeError("Missing required input [orientation]");

  setPose(pose);
  moveJoints();

  ROS_INFO("Trajectory Action IK Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !traj_client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    stop();
    ROS_ERROR("Trajectory Action IK aborted");
    return BT::NodeStatus::FAILURE;
  }
  
  ROS_INFO("Trajectory Action IK finished. ");
  return BT::NodeStatus::SUCCESS;
  
}


class SetGripperActionNode : public BT::AsyncActionNode, public BaxterArm {

  public:
    SetGripperActionNode(const std::string &name, const BT::NodeConfiguration &config) 
      : BT::AsyncActionNode(name, config), 
        BaxterArm("left") {
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort<std::string>("command"), 
              BT::InputPort<double>("position")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      halt_requested_ = true;
    }

  private:
    bool halt_requested_;

};

BT::NodeStatus SetGripperActionNode::tick() {

  ROS_INFO("Waiting for Gripper action server to start");
  if(!gripper_client_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact Gripper Action server");
    return BT::NodeStatus::FAILURE;
  }

  std::string cmd;
  if(getInput<std::string>("command", cmd)) {
    if (cmd=="open")
      BaxterArm::setGripper(100, 50);
    else if (cmd=="close")
      BaxterArm::setGripper(0, 50);
    else
      throw BT::RuntimeError("Invalid [command] input in SetGripper Action");
  }

  else{
    control_msgs::GripperCommandGoal cmd_pos;
    if(!getInput<control_msgs::GripperCommandGoal>("position", cmd_pos)){
      throw BT::RuntimeError("Missing required input [position]");
    }

    BaxterArm::setGripper(cmd_pos.command.position, cmd_pos.command.max_effort);
  }

  ROS_INFO("Gripper Action Server started. ");
  halt_requested_ = false;

  while(!halt_requested_ && !gripper_client_.waitForResult(ros::Duration(0.02)) && ros::ok()) {}

  if(halt_requested_) {
    BaxterArm::stopGripper();
    ROS_ERROR("Gripper Action aborted");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Gripper Action finished. ");
  return BT::NodeStatus::SUCCESS;
}
