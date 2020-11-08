#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/Pose.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBase {
  protected:
    MoveBaseClient ac_;
  
  public:
    MoveBase(std::string server_name="move_base") : ac_(server_name, true) {
      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
    }

    void GoTo(geometry_msgs::Pose pose, std::string frame_id="map") {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = frame_id;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = pose;

      ROS_INFO("Sending goal to action server.");
      ac_.sendGoal(goal,
                   boost::bind(&MoveBase::doneCB, this, _1),
                   MoveBaseClient::SimpleActiveCallback(),
                   MoveBaseClient::SimpleFeedbackCallback());
    }

    void doneCB(const actionlib::SimpleClientGoalState& state) {
      ROS_INFO("MoveBase action finished in state [%s]", state.toString().c_str());
      //ros::shutdown();
    }

    actionlib::SimpleClientGoalState getState() {
      return ac_.getState();
    }

};

/*
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_base_action_node");

  geometry_msgs::Pose pose;
  pose.position.x = -2;
  pose.position.y = 2;
  pose.orientation.w = 1;

  MoveBase mb;
  mb.GoTo(pose);

  ros::spin();
  return 0;
}
*/