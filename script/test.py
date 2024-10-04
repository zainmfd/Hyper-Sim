#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleStatus

class carla_to_twist:
    def __init__(self):
        self.role_name = 'ego_vehicle'
        self.control_sub = rospy.Subscriber("/carla/ego_vehicle/vehicle_status",CarlaEgoVehicleStatus,self.control_cb)
        self.twist_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=0)
        self.twist_msg = Twist()

    def control_cb(self,data):
        self.twist_msg.angular.z = -data.control.steer
        self.twist_msg.linear.x = data.control.throttle/5
        self.twist_pub.publish(self.twist_msg)



def main():
    rospy.init_node('hardware_simulation',anonymous=True)
    ctt = carla_to_twist()
    rospy.spin()
    


if __name__ == '__main__':
    main()
    