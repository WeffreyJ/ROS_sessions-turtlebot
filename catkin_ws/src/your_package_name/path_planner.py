import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    # Perform image processing to create a grid map
    grid_map = cv_image_processing(cv_image)

    # Perform path planning using Dijkstra's algorithm
    path = dijkstra_algorithm(grid_map)

    # Publish path as ROS message
    path_msg = convert_to_path_message(path)
    path_pub.publish(path_msg)

def cv_image_processing(cv_image):
    # Perform image processing to create a grid map
    # For example, thresholding to distinguish between walls and paths
    _, thresholded_image = cv2.threshold(cv_image, 127, 255, cv2.THRESH_BINARY)

    # Convert thresholded image to grid map format
    grid_map = convert_to_grid_map(thresholded_image)

    return grid_map

def convert_to_grid_map(image):
    # Convert OpenCV image to grid map format
    # For example, create a binary grid map where 0 represents paths and 1 represents walls
    grid_map = np.where(image == 255, 1, 0)
    return grid_map

def dijkstra_algorithm(grid_map):
    # Implement Dijkstra's algorithm to compute the shortest path
    # This is a simplified example of Dijkstra's algorithm
    # Replace it with your own implementation

    # Example code for a random path (not Dijkstra's algorithm)
    rows, cols = grid_map.shape
    start_node = (0, 0)
    end_node = (rows - 1, cols - 1)
    path = [start_node]

    while path[-1] != end_node:
        current_node = path[-1]
        next_node = (current_node[0] + 1, current_node[1])
        if next_node[0] < rows and grid_map[next_node] == 0:
            path.append(next_node)
        else:
            next_node = (current_node[0], current_node[1] + 1)
            if next_node[1] < cols and grid_map[next_node] == 0:
                path.append(next_node)
            else:
                # If no valid next node, break the loop
                break

    return path

def convert_to_path_message(path):
    # Convert computed path to ROS Path message
    # You need to implement this function according to your ROS message format
    pass

import imageio

def main():
    rospy.init_node('path_planner')
    
    # Specify the path to your local GIF file here
    image_path = '/workspaces/labs-WeffreyJ/searching_map_HW/maps/my_maze2.gif'
    
    gif = imageio.get_reader(image_path)
    cv_image = gif.get_data(0)  # Get the first frame
    
    rospy.Subscriber('/image_topic', Image, image_callback)
    global path_pub
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    main()
