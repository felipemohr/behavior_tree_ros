#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main (int argc, char** argv) {

  ros::init(argc, argv, "move_circles");
  ros::NodeHandle nh;

  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 10);

  geometry_msgs::Twist vel;
  vel.linear.x = 1.0;
  vel.angular.z = 0.4;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    
    vel_pub.publish(vel);
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}
