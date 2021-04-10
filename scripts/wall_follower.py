#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class WallFollower(object):
    """This node makes the robot follow the wall"""

    def __init__(self):
        # Initiate node, publisher, subscriber
        rospy.init_node("wall_follower")
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)
        self.wall_dist = 0.3 # tolerable distance from wall
        self.turn_dist = 1.0 # tolerable distance before turning
        self.found_wall = False # found wall or not
        self.parallel_k = 0.3 # proportional constant for staying parallel to wall
        rospy.sleep(1)
    
    # Driver code for tracing wall
    def process_scan(self, data):
        # Regions (front, right)
        front_region = data.ranges[0:10] + data.ranges[350:359]
        right_region = data.ranges[240:280]
        front_min = min(min(front_region), 5)
        right_min = min(min(right_region), 5)
        
        # Find wall
        if not self.found_wall:
            # Finding wall
            if front_min > self.turn_dist and right_min > self.wall_dist:
                print("Finding wall")
                self.twist.linear.x = 0.2
                self.twist.angular.z = -0.01
            else:
                self.found_wall = True
        else:
            # Turning 
            if front_min <= self.turn_dist: 
                self.twist.linear.x = 0.1
                ang_vel = 1
                self.twist.angular.z = ang_vel
                print("Turning", ang_vel)
            else:
                # Going straight, error correcting to stay staight
                self.twist.linear.x = 0.2
                err = self.wall_dist - right_min
                ang_vel = err * self.parallel_k
                self.twist.angular.z = ang_vel
                print("Forward", ang_vel)
        self.twist_pub.publish(self.twist)

    # Keep the robot running 
    def run(self):
        rospy.spin()

if __name__ == '__main__':      
    # Declare a node and run it.
    node = WallFollower()
    node.run()


