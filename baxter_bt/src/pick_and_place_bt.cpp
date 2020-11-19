#include "nodes.cpp"


int main(int argc, char** argv) {

  ros::init(argc, argv, "pick_and_place_tree");
  
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<TrajectoryAction>("MoveJoints");
  factory.registerNodeType<GripperAction>("SetGripper");

  auto tree = factory.createTreeFromFile("src/behavior_tree_ros/baxter_bt/trees/pick_and_place_tree.xml");

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
