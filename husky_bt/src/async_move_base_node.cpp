#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/Pose.h"


static const char* xml_text = R"(
  <root>
    <BehaviorTree>
      <Sequence>
        <MoveBase pose="-3 2 0 0 0 0 1"/>
        <MoveBase pose="1 1 0 0 0 0 1"/>
      </Sequence>
    </BehaviorTree>
  </root>
)";


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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


class MoveBaseAction : public BT::AsyncActionNode {
  public:
    MoveBaseAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), ac_("move_base", true) {
      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
    }

    static BT::PortsList providedPorts() {
      return {BT::InputPort<geometry_msgs::Pose>("pose")};
    }

    BT::NodeStatus tick() override;

    void halt() override {
      _halt_requested.store(true);
    }
    void doneCB(const actionlib::SimpleClientGoalState& state) {
      ROS_INFO("MoveBase action finished in state [%s]", state.toString().c_str());
      ros::shutdown();
    }

  private:
    std::atomic_bool _halt_requested;
    MoveBaseClient ac_;


};

BT::NodeStatus MoveBaseAction::tick() {
  geometry_msgs::Pose pose;
  if(!getInput<geometry_msgs::Pose>("pose", pose))
    throw BT::RuntimeError("Missing required input [pose]");

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose;

  ROS_INFO("Sending goal to action server.");
  ac_.sendGoal(goal,
                boost::bind(&MoveBaseAction::doneCB, this, _1),
                MoveBaseClient::SimpleActiveCallback(),
                MoveBaseClient::SimpleFeedbackCallback());

  ROS_INFO("[ MoveBase: STARTED ]. goal: x=%.1f y=%.1f\n", pose.position.x, pose.position.y);
  _halt_requested.store(false);

  while(!_halt_requested && ros::ok()) { }

  ROS_INFO("ModeBase: FINISHED");
  return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "tree_node");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MoveBaseAction>("MoveBase");

  auto tree = factory.createTreeFromText(xml_text);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
