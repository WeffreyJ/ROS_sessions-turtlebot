#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def linear_movement(publisher, loop_rate, travel_distance, travel_speed):
    command = Twist()
    command.linear.x = travel_speed
    command.angular.z = 0

    move_time = travel_distance / travel_speed
    time_begin = rospy.Time.now()

    while (rospy.Time.now() - time_begin).to_sec() < move_time:
        publisher.publish(command)
        loop_rate.sleep()

    command.linear.x = 0
    publisher.publish(command)

def rotate_robot(publisher, loop_rate, rotation_angle, rotation_velocity):
    command = Twist()
    command.linear.x = 0
    command.angular.z = rotation_velocity

    rotation_duration = rotation_angle / rotation_velocity
    time_begin = rospy.Time.now()

    while (rospy.Time.now() - time_begin).to_sec() < rotation_duration:
        publisher.publish(command)
        loop_rate.sleep()

    command.angular.z = 0
    publisher.publish(command)

def execute_square_path():
    rospy.init_node('robot_square_movement', anonymous=False)
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    path_length = 0.5
    move_speed = 0.4  # Adjusted linear speed to 0.4 m/s
    angular_speed = 0.5

    while not rospy.is_shutdown():
        for _ in range(4):
            linear_movement(cmd_vel_publisher, rate, path_length, move_speed)
            rotate_robot(cmd_vel_publisher, rate, 1.5708, angular_speed)

if __name__ == '__main__':
    try:
        execute_square_path()
    except rospy.ROSInterruptException:
        pass
