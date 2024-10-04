#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np


class Transformer():
    def __init__(self):
        rospy.init_node('transformer_node', anonymous=True)
    
        # Subscribers
        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self.transform_odometry)
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odometry/transformed', Odometry, queue_size=10)
        
        print("Odometry Transformer Started...")
            
    def transform_odometry(self, data: Odometry):
        '''
        Rotates odometry about the y-axis and publishes to /odometry/transformed
        '''
        pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        
        transformed_pose = self.rotate_about_axis(pose, -0.40, "y")
        
        transformed_odom = Odometry()
        
        transformed_odom.header.frame_id="odom"
        transformed_odom.child_frame_id = "base_link"
        transformed_odom.pose.pose.position.x = transformed_pose[0]
        transformed_odom.pose.pose.position.y = transformed_pose[1]
        transformed_odom.pose.pose.position.z = transformed_pose[2]
        transformed_odom.pose.pose.orientation = data.pose.pose.orientation
        
        self.odom_pub.publish(transformed_odom)
        
    def rotate_about_axis(self, pose, angle, axis = "y"):
        '''
        pose : [x, y, z]
        angle in radians
        axis : x or y or z
        '''
        rot_mat = None
        if axis == "x":
            rot_mat = np.array([[1, 0, 0],
                                [0, np.cos(angle), -np.sin(angle)],
                                [0, np.sin(angle), np.cos(angle)]])
        elif axis == "y":
            rot_mat = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
        elif axis == "z":
            rot_mat = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])

        transformed_pose = rot_mat.dot(pose)
        
        return transformed_pose
    
def main():
    transformer = Transformer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass