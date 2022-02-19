#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_controller");
  ros::NodeHandle nh;

  // TODO: Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "husky_velocity_controller/cmd_vel", 100
  );

  // Allows the publisher to publish at 10 Hz.
  ros::Rate rate(10);

  geometry_msgs::Twist msg;
  msg.linear.x = 0.75;
  msg.angular.z = 0.5;
  while(ros::ok()) {
    pub.publish(msg);
    rate.sleep();
  }

  return(0);
}
