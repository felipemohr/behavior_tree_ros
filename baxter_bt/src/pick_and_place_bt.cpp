#include "nodes.cpp"


int main(int argc, char** argv) {

  ros::init(argc, argv, "pick_and_place_tree");
/*
  BaxterArm baxter_arm;
  geometry_msgs::Pose pose;
  pose.position.x = 0.75; 
  pose.position.y = -0.05; 
  pose.position.z = -0.021; 
  pose.orientation.x = -0.025;
  pose.orientation.y = 1;
  pose.orientation.z = 0.007;
  pose.orientation.w = 0.005;
  baxter_arm.setPose(pose);
  baxter_arm.moveJoints();
  baxter_arm.wait();

  pose.position.x = 0.95; 
  baxter_arm.setPose(pose);
  baxter_arm.moveJoints();
  baxter_arm.wait();

  geometry_msgs::Pose offset;
  offset.position.x = 0.2; 
  offset.position.y = 0.0; 
  offset.position.z = 0.0;
  baxter_arm.offsetPose(offset);
  baxter_arm.moveJoints();
  baxter_arm.wait();
*/
  
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<MoveJointsActionNode>("MoveJoints");
  factory.registerNodeType<MoveToPoseActionNode>("MoveToPose");
  factory.registerNodeType<SetGripperActionNode>("SetGripper");

  auto tree = factory.createTreeFromFile("src/behavior_tree_ros/baxter_bt/trees/pick_and_place_tree.xml");

  BT::StdCoutLogger logger_cout(tree);
  BT::FileLogger logger_file(tree, "src/behavior_tree_ros/baxter_bt/loggers/bt_trace.fbl");
  BT::MinitraceLogger logger_minitrace(tree, "src/behavior_tree_ros/baxter_bt/loggers/bt_trace.json");
  BT::PublisherZMQ publisher_zmq(tree);

  BT::printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
