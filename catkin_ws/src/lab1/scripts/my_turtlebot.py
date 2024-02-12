#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import cos, sin, pi

class MyTurtle:
    def __init__(self):
        rospy.init_node('my_turtlebot_node')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angular_speed = 0.7

    def stop(self):
        twist_cmd = Twist()
        self.cmd_pub.publish(twist_cmd)

    def drive_in_circle(self, radius: float):
        linear_speed = self.angular_speed * radius
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_speed
        twist_cmd.angular.z = self.angular_speed
        self.cmd_pub.publish(twist_cmd)

def main():
    turtle = MyTurtle()

    # Drive in a circle with radius 0.5m indefinitely
    while not rospy.is_shutdown():
        turtle.drive_in_circle(radius=0.5)

    # Stop the turtlebot
    turtle.stop()

if __name__ == '__main__':
    main()
