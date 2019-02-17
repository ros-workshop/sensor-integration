#include <ros/ros.h>
// TODO: Include the appropriate header for sending Twist messages.
#include  <stdlib.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_controller");
  ros::NodeHandle nh;

  // TODO: Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.

  // Allows the publisher to publish at 10 Hz.
  ros::Rate rate(10);

  while(ros::ok()) {
    // TODO: Make Husky move in circles by publishing to husky_velocity_controller/cmd_vel topic.

    rate.sleep();
  }
}
