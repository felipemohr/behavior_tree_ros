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


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "tree");
  
  MoveBase mb;

  BT::BehaviorTreeFactory factory;

  auto MoveToWrapperWithLambda = [&mb](BT::TreeNode &parent_node) -> BT::NodeStatus {
    geometry_msgs::Pose pose;
    parent_node.getInput("pose", pose);

    mb.GoTo(pose);

    //if (mb.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    //  return BT::NodeStatus::RUNNING;
    //else
      return mb.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

  };


  BT::PortsList ports = { BT::InputPort<geometry_msgs::Pose>("pose") };
  factory.registerSimpleAction("MoveBase", MoveToWrapperWithLambda, ports);

  auto tree = factory.createTreeFromText(xml_text);
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
