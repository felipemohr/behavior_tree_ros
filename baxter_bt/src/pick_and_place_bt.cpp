#include "nodes.cpp"

void f();








int main(int argc, char** argv) {

  ros::init(argc, argv, "pick_and_place_tree");
  ros::NodeHandle nh;
  
  ros::ServiceClient client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");

  baxter_core_msgs::SolvePositionIK srv;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.15;
  pose.pose.position.z = -0.129;
  pose.pose.orientation.x = -0.02496;
  pose.pose.orientation.y = 0.99965;
  pose.pose.orientation.z = 0.00738;
  pose.pose.orientation.w = 0.00486;

  srv.request.pose_stamp.push_back(pose);
  //srv.request.seed_mode = baxter_core_msgs::SolvePositionIKRequest::SEED_USER;

  Trajectory traj;
  if(client.call(srv)){
    std::vector<double> point;
    sensor_msgs::JointState js;

    js = srv.response.joints[0];
    point.push_back(js.position[0]);
    point.push_back(js.position[1]);
    point.push_back(js.position[2]);
    point.push_back(js.position[3]);
    point.push_back(js.position[4]);
    point.push_back(js.position[5]);
    point.push_back(js.position[6]);


    
    traj.setPoint(point, 7.0);
    traj.start();
    ROS_INFO("Done");
  }
  else {
    ROS_ERROR("IK Service failed");
    if(!srv.response.isValid[0])
      ROS_ERROR("Out of range");
    return 1;
  }

  return 0;
}











void f() {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<TrajectoryAction>("MoveJoints");
  factory.registerNodeType<GripperAction>("SetGripper");

  auto tree = factory.createTreeFromFile("src/behavior_tree_ros/baxter_bt/trees/pick_and_place_tree.xml");

  BT::StdCoutLogger logger_cout(tree);
  BT::FileLogger logger_file(tree, "src/behavior_tree_ros/baxter_bt/loggers/bt_trace.fbl");
  BT::MinitraceLogger logger_minitrace(tree, "src/behavior_tree_ros/baxter_bt/loggers/bt_trace.json");
  #ifdef ZMQ_FOUND
    BT::PublisherZMQ publisher_zmq(tree);
  #endif

  BT::printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
