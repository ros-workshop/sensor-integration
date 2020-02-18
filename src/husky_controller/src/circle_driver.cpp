#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
// TODO: Include the appropriate header for sending Twist messages.
#include  <stdlib.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_controller");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  // TODO: Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.
  // Allows the publisher to publish at 10 Hz.
  ros::Rate rate(10);

  while(ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear={2.0,0.0,0.0};
    msg.angular={0.0,0.0,1.8};
    pub.publish(msg);
    // TODO: Make Husky move in circles by publishing to husky_velocity_controller/cmd_vel topic.

    rate.sleep();
  }
}
