#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import radians

class DriveSquare(object):
    """This node publishes ROS messages to make the robot drive in a square"""
    def __init__(self, forward_speed, forward_duration, turn_speed):
        """The constructor allows the user to specify linear speed, angular speed"""
        """and time it takes to drive one side of the square. Larger speeds mean more noise"""
        # Init node and setup publisher
        rospy.init_node('drive_in_square')
        self.drive_square_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Init parameters
        self.speed = forward_speed 
        self.forward_duration = forward_duration
        self.turn_speed = turn_speed

    # Method to make the robot move forward for a set amount of time
    def goForward(self):
        forward_msg = Twist( 
                linear=Vector3(self.speed, 0, 0),
                angular=Vector3(0, 0, 0))
        rospy.sleep(1)
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - init_time < self.forward_duration:
            # Drive forward for a set amount of time
            self.drive_square_pub.publish(forward_msg)
    
    # Method to make the robot turn 90 degrees
    def turn(self):
        turn_msg = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, self.turn_speed))
        rospy.sleep(1)
        current_angle = 0
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and current_angle < radians(90):
            # Continue turning until the current angle (speed * time elapsed) equals a 90 degree turn
            self.drive_square_pub.publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.turn_speed * (t1 - init_time)

        # Stop turning
        turn_msg.angular.z = 0
        self.drive_square_pub.publish(turn_msg)
    
    # Method to drive in one square
    def driveSquare(self):
        for i in range(0, 4):
            self.goForward()
            self.turn()
    
    # Method to continuously drive in squares
    def run(self):
        while not rospy.is_shutdown():
            self.driveSquare()

if __name__ == '__main__':
    node = DriveSquare(0.1, 8, 0.2)
    # Drive only one square for recording 
    node.driveSquare()

