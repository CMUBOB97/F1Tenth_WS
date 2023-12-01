#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion  # sudo apt install ros-foxy-tf-transformations 
import time
import csv

filename = "/home/jmoney3/F1Tenth_WS/src/mpc/waypoint.csv"

class logWaypoint(Node):
    def __init__(self):
        super().__init__('log_waypoint')
        self.odom_subscriber = self.create_subscription(Odometry, 'odom',self.pose_back, 10)
        self.lastTime = 0

    def pose_back(self, msg):
        if msg.header.stamp.sec - self.lastTime > 0.25:
            print("saving waypoint" , msg.header.stamp.sec)
            quaternion = np.array([msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])

            euler = euler_from_quaternion(quaternion)
            speed = msg.twist.twist.linear.x
            with open(filename, 'a') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([msg.pose.pose.position.x,
                                            msg.pose.pose.position.y,
                                            euler[2],
                                            speed])
                
            self.lastTime = msg.header.stamp.sec

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = logWaypoint()
    rclpy.spin(waypoint_logger)
    rclpy.shutdown()

if __name__ == '__main__':
    print('Saving waypoints...')
    main()
            
            
