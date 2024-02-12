#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class DancingTurtleBot:
    def __init__(self):
        rospy.init_node('dancing_turtlebot', anonymous=False)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Set loop rate

        self.infinity_path_distance = 1.0  # Total distance for infinity path in meters
        self.linear_path_length = 2.0  # Length of the linear path in meters
        self.linear_speed = 0.2  # Linear speed in m/s
        self.angular_speed = 0.5  # Angular speed in rad/s
        self.movement_command = Twist()  # Initialize Twist object for movement commands

    def dance_and_trace_infinity(self):
        # Loop to trace the infinity path and sporadically rotate
        while not rospy.is_shutdown():
            # Trace the first loop of the infinity path
            self.trace_linear_path(0.5)

            # Rotate for dancing effect
            self.rotate_dance()

            # Trace the second loop of the infinity path
            self.trace_linear_path(0.5)

            # Rotate for dancing effect
            self.rotate_dance()

    def trace_linear_path(self, duration):
        start_time = rospy.Time.now().to_sec()
        distance_traveled = 0.0

        while distance_traveled < self.linear_path_length:
            # Calculate the remaining distance to the end of the linear path
            remaining_distance = self.linear_path_length - distance_traveled

            # Adjust loop duration based on remaining distance
            current_loop_duration = min(duration, remaining_distance / self.linear_speed)

            # Update the movement command
            self.movement_command.linear.x = self.linear_speed
            self.movement_command.angular.z = 0.0  # No angular rotation during linear movement

            # Publish the movement command
            self.cmd_vel_pub.publish(self.movement_command)
            self.rate.sleep()

            # Update distance traveled
            distance_traveled += self.linear_speed * current_loop_duration

        # Stop the TurtleBot
        self.movement_command.linear.x = 0.0
        self.movement_command.angular.z = 0.0
        self.cmd_vel_pub.publish(self.movement_command)
        self.rate.sleep()

    def rotate_dance(self):
        rotation_duration = rospy.Duration.from_sec(1.0)  # Rotate for 1 second
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time) < rotation_duration:
            self.movement_command.linear.x = 0.0
            self.movement_command.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.movement_command)
            self.rate.sleep()

        # Stop the TurtleBot after rotating
        self.movement_command.linear.x = 0.0
        self.movement_command.angular.z = 0.0
        self.cmd_vel_pub.publish(self.movement_command)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        dancing_turtlebot = DancingTurtleBot()
        dancing_turtlebot.dance_and_trace_infinity()
    except rospy.ROSInterruptException:
        pass
