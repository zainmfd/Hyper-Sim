#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_carla_callback(data):
    # Callback for the /cmd_vel_carla topic
    rospy.loginfo("Received /cmd_vel_carla message: %s", data)
    # Republish the data to /cmd_vel
    cmd_vel_pub.publish(data)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('cmd_vel_republisher', anonymous=True)

    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to the /cmd_vel_carla topic
    rospy.Subscriber('/cmd_vel_carla', Twist, cmd_vel_carla_callback)

    # Keep the node running
    rospy.spin()
