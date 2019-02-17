#include <ros/ros.h>
// Include the appropriate header to send Twist messages.
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_controller");
  ros::NodeHandle nh;

  // Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 100);

  // Allows the publisher to publish at 10 Hz.
  ros::Rate rate(10);

  while(ros::ok()) {
    // Make Husky move in circles by publishing to husky_velocity_controller/cmd_vel topic.
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 2.0;
    rate.sleep();
  }
}
