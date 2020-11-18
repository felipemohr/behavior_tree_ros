#include <ros/ros.h>
#include "baxter_core_msgs/JointCommand.h"
#include "actionlib/server/simple_action_server.h"
#include "baxter_bt/BaxterJointCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/JointTrajectoryAction.h"

class BaxterJointCommandAction {
  
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<baxter_bt::BaxterJointCommandAction> as_;
    baxter_bt::BaxterJointCommandFeedback feedback_;
    baxter_bt::BaxterJointCommandResult result_;

    std::string action_name_;
    baxter_core_msgs::JointCommand cmd_;


  public:
    BaxterJointCommandAction(std::string name) 
      : as_(nh_, name, boost::bind(&BaxterJointCommandAction::executeCB, this, _1), false), action_name_(name) {
        as_.start();
    }

    ~BaxterJointCommandAction(void) {}

    void executeCB(const baxter_bt::BaxterJointCommandGoalConstPtr &goal) {
      ros::Rate r(5);
      bool success = true;

      
    }


};

