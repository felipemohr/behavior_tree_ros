#include "behaviortree_cpp_v3/bt_factory.h"
#include "actions_classes.cpp"


namespace BT {
  template <> std::vector<double> convertFromString(StringView key) {

    auto input = BT::splitString(key, ' ');

    std::vector<double> point;
    for(auto j:input)
      point.push_back(convertFromString<double>(j));

    return point;
  }

}


class TrajectoryAction : public BT::AsyncActionNode, public Trajectory {

  public:
    TrajectoryAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), Trajectory("robot/limb/left/follow_joint_trajectory") {
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

  ROS_INFO("Waiting for action server to start");
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
