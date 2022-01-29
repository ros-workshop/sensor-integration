#!/usr/bin/env python3
import rospy
# TODO: Include the appropriate library for sending Twist messages.


def main():
    rospy.init_node(name='husky_controller')
    
    # TODO: Create a publisher that publishes to husky_velocity_controller/cmd_vel topic.
    
    # Allows the publisher to publish at 10 Hz.
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # TODO: Make Husky move in circles by publishing to husky_velocity_controller/cmd_vel topic.

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass