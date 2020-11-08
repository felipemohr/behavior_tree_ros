#include "behaviortree_cpp_v3/bt_factory.h"
#include "move_base_class_action.cpp"

namespace BT {
  template <> Pose2D convertFromString(StringView key) {
    auto coords = BT::splitString(key, ' ');

    if (coords.size() != 3)
      throw RuntimeError("Invalid Input.");
    else {
      Pose2D pose;
      pose.x = convertFromString<double>(coords[0]);
      pose.y = convertFromString<double>(coords[1]);
      pose.theta = convertFromString<double>(coords[2]);
      return pose;
    }
  }
}

class MBAction : public BT::AsyncActionNode, public MoveBase {
  public:
    MBAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), MoveBase("move_base") {
    }
   
    static BT::PortsList providedPorts() {
      return {BT::InputPort<Pose2D>("pose")};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override {
      _halt_requested = true;
    }

  private:
    bool _halt_requested;

};

BT::NodeStatus MBAction::tick() {

  ROS_INFO("Waiting for action server to start.");
  if (!ac_.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move base server");
    return BT::NodeStatus::FAILURE;
  }
  
  Pose2D pose;
  if(!getInput<Pose2D>("pose", pose))
    throw BT::RuntimeError("Missing required input [pose]");

  MoveBase::goTo(pose);

  ROS_INFO("MoveBase started. \ngoal: x=%.1f y=%.1f theta=%.2f\n", pose.x, pose.y, pose.theta);
  _halt_requested = false;

  while(!_halt_requested && !ac_.waitForResult(ros::Duration(0.02)) && ros::ok()) { }

  if (_halt_requested) {
    ac_.cancelAllGoals();
    ROS_ERROR("MoveBase aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveBase failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("MoveBase finished. Target reached");
  return BT::NodeStatus::SUCCESS;

}
