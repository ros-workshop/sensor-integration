# Solution: Making Husky move in circles

Sample code to make Husky move in a circular path. 

A `C++` or `python` solution can be used

## C++
[`src/husky_controller/src/circle_driver.cpp`](src/husky_controller/src/circle_driver.cpp)
```cpp
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
    pub.publish(msg);
    rate.sleep();
  }
}
```

[`src/husky_controller/launch/circle_driver.launch`](src/husky_controller/launch/circle_driver.launch)

```xml
<?xml version="1.0"?>
<launch>
	<node pkg="husky_controller" name="circle_driver" type="circle_driver" />
</launch>
```


## Python
[`src/husky_controller/src/circle_driver_python`](src/husky_controller/src/circle_driver_python)

```python
#!/usr/bin/env python3
import rospy
# Include the appropriate library for sending Twist messages.
from geometry_msgs.msg import Twist


def main():
    rospy.init_node(name="husky_controller")

    # Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.
    pub = rospy.Publisher(name="husky_velocity_controller/cmd_vel", 
                          data_class=Twist, 
                          queue_size=100)

    # Allows the publisher to publish at 10 Hz.
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # Make Husky move in circles by publishing to husky_velocity_controller/cmd_vel topic.
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 2.0
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

```

```xml
<?xml version="1.0"?>
<launch>
	<node pkg="husky_controller" name="circle_driver_python" type="circle_driver_python" />
</launch>
```
