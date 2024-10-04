#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from carla_msgs.msg import CarlaEgoVehicleStatus

class Transformer():
    def __init__(self, wheelbase = 0.8, steer_max=1.57):
        rospy.init_node('transformer_node', anonymous=True)
    
        # Subscribers
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.ackermann_to_twist)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        
        self.wheelbase = wheelbase
        self.steer_max = steer_max # maximum steering angle
                
        print("Ackermann to Twist Transformer Started...")
            
    def ackermann_to_twist(self, data: CarlaEgoVehicleStatus):
        '''
            Converts Ackermann steering commands to Twist 
        '''
        delta = data.control.steer * self.steer_max # steering angle
        v = data.velocity
        w = np.tan(delta)*v/self.wheelbase
        
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        
        self.cmd_vel_pub.publish(cmd_vel)    
    
def main():
    transformer = Transformer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass