#include "nodes.cpp"


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "tree");
  
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<MBAction>("MoveBase");

  auto tree = factory.createTreeFromFile("src/behavior_tree_ros/husky_bt/trees/move_base_tree.xml");
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
