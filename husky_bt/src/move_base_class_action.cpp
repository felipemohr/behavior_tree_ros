#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"

struct Pose2D {
  double x, y, theta;
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBase {
  protected:
    MoveBaseClient ac_;
  
  public:
    MoveBase(std::string server_name="move_base") : ac_(server_name, true) {
      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
    }

    void goTo(Pose2D pose, std::string frame_id="map") {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = frame_id;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = pose.x;
      goal.target_pose.pose.position.y = pose.y;
      tf::Quaternion rot = tf::createQuaternionFromYaw(pose.theta);
      tf::quaternionTFToMsg(rot, goal.target_pose.pose.orientation);

      ROS_INFO("Sending goal to action server.");
      ac_.sendGoal(goal);
    }

};
