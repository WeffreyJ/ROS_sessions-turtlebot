#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, pow, pi

class AutonomousRobot:
    def __init__(self):
        rospy.init_node('autonomous_navigation', anonymous=False)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.destination_listener = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navigate_to_destination)
        self.transform_listener = tf.TransformListener()

        self.loop_rate = rospy.Rate(10)  # Define loop rate as 10 Hz

    def adjust_angle(self, angle):
        """
        Adjusts an angle to fall within the range of [-pi, pi].
        """
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def navigate_to_destination(self, destination):
        # Extract position and orientation from the destination message
        destination_pos = destination.pose.position
        destination_orient = destination.pose.orientation
        _, _, destination_yaw = tf.transformations.euler_from_quaternion([destination_orient.x, destination_orient.y, destination_orient.z, destination_orient.w])

        navigation_command = Twist()

        # Continue towards the destination unless interrupted
        while not rospy.is_shutdown():
            try:
                # Retrieve current position and orientation
                (current_position, current_orientation) = self.transform_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                _, _, current_yaw = tf.transformations.euler_from_quaternion(current_orientation)

                # Calculate the distance to the destination
                distance = sqrt(pow(destination_pos.x - current_position[0], 2) + pow(destination_pos.y - current_position[1], 2))

                # Calculate the angle to the destination
                desired_yaw = atan2(destination_pos.y - current_position[1], destination_pos.x - current_position[0])

                # Calculate the necessary yaw adjustment
                yaw_diff = self.adjust_angle(desired_yaw - current_yaw)

                # Decide to rotate or move forward based on yaw difference
                if abs(yaw_diff) > 0.1:
                    navigation_command.linear.x = 0.0
                    navigation_command.angular.z = 0.5 if yaw_diff > 0 else -0.5
                elif distance > 0.1:
                    navigation_command.linear.x = 0.4  # Set linear speed to 0.4 m/s
                    navigation_command.angular.z = 0
                
                self.cmd_vel_pub.publish(navigation_command)
                self.loop_rate.sleep()

                # Stop if close enough to the destination
                if distance < 0.1:
                    navigation_command = Twist()  # Stop movement
                    self.cmd_vel_pub.publish(navigation_command)
                    rospy.loginfo("Reached destination.")
                    break

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

if __name__ == '__main__':
    try:
        nav_robot = AutonomousRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
