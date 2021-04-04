#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to person
distance = 0.5

class PersonFollower(object):

    def __init__(self):
        rospy.init_node("person_follower")

        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


