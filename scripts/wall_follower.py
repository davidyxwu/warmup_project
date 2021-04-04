#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# Specify distance of how close we get to the wall
distance = 0.5

class WallFollower(object):
    def __init__(self):
        rospy.init_node("wall_follower")
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)

