#include "behaviortree_cpp_v3/bt_factory.h"
#include "move_base_class_action.cpp"
#include "geometry_msgs/Pose.h"


static const char* xml_text = R"(
  <root>
    <BehaviorTree>
      <Sequence>
        <MoveBase pose="-3 2 0 0 0 0 1"/>
        <MoveBase pose="0 0 0 0 0 0 1"/>
      </Sequence>
    </BehaviorTree>
  </root>
)";


namespace BT {
  template <> geometry_msgs::Pose convertFromString(StringView key) {
    auto coords = BT::splitString(key, ' ');

    if (coords.size() != 7)
      throw RuntimeError("Invalid Input. Use: X Y Z x y z w");
    else {
      geometry_msgs::Pose pose;
      pose.position.x = convertFromString<double>(coords[0]);
      pose.position.y = convertFromString<double>(coords[1]);
      pose.position.z = convertFromString<double>(coords[2]);
      pose.orientation.x = convertFromString<double>(coords[3]);
      pose.orientation.y = convertFromString<double>(coords[4]);
      pose.orientation.z = convertFromString<double>(coords[5]);
      pose.orientation.w = convertFromString<double>(coords[6]);
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
      return {BT::InputPort<geometry_msgs::Pose>("pose")};
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
  
  geometry_msgs::Pose pose;
  if(!getInput<geometry_msgs::Pose>("pose", pose))
    throw BT::RuntimeError("Missing required input [pose]");

  GoTo(pose);

  ROS_INFO("[ MoveBase: STARTED ]. goal: x=%.1f y=%.1f\n", pose.position.x, pose.position.y);
  _halt_requested = false;

  while(!_halt_requested && !ac_.waitForResult(ros::Duration(0.02))) { }

  if (_halt_requested) {
    ac_.cancelAllGoals();
    ROS_ERROR("MoveBase aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveBase failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("ModeBase finished. Target reached");
  return BT::NodeStatus::SUCCESS;

}


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "tree");
  
  //MoveBase mb;

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<MBAction>("MoveBase");

  auto tree = factory.createTreeFromText(xml_text);
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
