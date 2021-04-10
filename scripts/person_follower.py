#!/usr/bin/env python3
import rospy
from math import radians
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class PersonFollower(object):
    """This node follows an object within a distance of 3.0 meters"""
    def __init__(self):
        # init rospy node
        rospy.init_node("person_follower")
        
        # Declare our node as a subscriber to /scan topic 
        # Set self.process_scan as the function to be used for callback
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        
        # Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Set up parameters
        lin = Vector3()
        ang = Vector3()
        self.ang_constant = 0.03
        self.lin_k = 0.3
        self.distance = 0.5
        self.twist = Twist(linear=lin, angular=ang)
        rospy.sleep(1)
    
    # Find the degree the robot needs to turn by finding index of min element in ranges array
    # Closest distance, degree range of -180 <= degree <= 180
    def turn_degree(self, data):
        degree = data.ranges.index(min(data.ranges))
        # If object is in back of robot, use 180 degree measure 
        # so robot does not switch between positive and negative angular velocity
        if 170 <= degree <= 190:
            adj_degree = 180
        elif degree > 180:
            adj_degree = degree - 360
        else:
            adj_degree = degree
        return [degree, adj_degree]
        
    def process_scan(self, data):
        # Desired set-angle is the degree we need to turn to 
        sensor_info = self.turn_degree(data)
        turn_index = sensor_info[0]
        set_angle = sensor_info[1]
        # Object is in front of robot, stop turning
        if set_angle <= -170 or (set_angle >= 0 and set_angle <= 10):
            self.twist.angular.z = 0
        else:
            # Proportional control to target 0 degrees
            self.twist.angular.z = set_angle * self.ang_constant
        # If object is within LaserScan range
        if data.ranges[turn_index] >= self.distance and data.ranges[turn_index] <= 3.5:
            if data.ranges[turn_index] >= 3.0:
                # If distance >= 3.0, go slow to avoid going out of range
                self.twist.linear.x = 0.1
            else:
                # If distance < 3.0, adjust speed according proportional control
                err = data.ranges[turn_index] - self.distance
                lin_vel = err * self.lin_k
                self.twist.linear.x = lin_vel
        else:
            # Stop once robot is within distance threshold
            self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)
        print(data.ranges[turn_index], set_angle, self.twist.linear.x, self.twist.angular.z)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()


