import random
import time
from PIL import Image
import math
import matplotlib.pyplot as plt

# Initialize start and end points
start = (0, 0)  # Replace with actual starting point coordinates
end = (599, 350)  # Replace with actual ending point coordinates

# Colors for visualization
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)

# Constants
MAX_ITERATIONS = 100000
MAX_STEP_SIZE = 700.225

# Data structures for the algorithm
tree = {}

# k-d tree node
class Node:
    def __init__(self, point):
        self.point = point
        self.left = None
        self.right = None

def build_kd_tree(points, depth=0):
    if not points:
        return None

    k = len(points[0])  # Dimensionality of the points
    axis = depth % k

    points.sort(key=lambda x: x[axis])
    median = len(points) // 2

    node = Node(points[median])
    node.left = build_kd_tree(points[:median], depth + 1)
    node.right = build_kd_tree(points[median + 1:], depth + 1)

    return node

def nearest_neighbor_search(tree, target):
    best = None
    best_dist = float('inf')

    def recursive_search(node, depth=0):
        nonlocal best, best_dist

        if node is None:
            return

        axis = depth % len(target)
        if target[axis] < node.point[axis]:
            next_node = node.left
            opposite_node = node.right
        else:
            next_node = node.right
            opposite_node = node.left

        recursive_search(next_node, depth + 1)

        node_dist = sum((node.point[i] - target[i]) ** 2 for i in range(len(target)))
        if node_dist < best_dist:
            best = node
            best_dist = node_dist

        if (target[axis] - node.point[axis]) ** 2 < best_dist:
            recursive_search(opposite_node, depth + 1)

    recursive_search(tree)
    return best.point

# RRT search algorithm
def rrt_search(image, start, goal):
    tree[start] = None

    # Build k-d tree from the tree nodes
    points = list(tree.keys())
    kd_tree = build_kd_tree(points)

    distances_to_goal = []
    for iteration in range(MAX_ITERATIONS):
        # Randomly sample a point in the environment
        rand_point = (random.randint(0, image.size[0] - 1), random.randint(0, image.size[1] - 1))

        # Find the nearest node in the tree using k-d tree
        nearest_node = nearest_neighbor_search(kd_tree, rand_point)

        # Extend the tree towards the random point
        dx = rand_point[0] - nearest_node[0]
        dy = rand_point[1] - nearest_node[1]
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist != 0:
            new_node = (nearest_node[0] + int(MAX_STEP_SIZE * dx / dist), nearest_node[1] + int(MAX_STEP_SIZE * dy / dist))
        else:
            # Handle the case when dist is zero (avoid division by zero)
            # For example, set a default value for the new node coordinates
            new_node = nearest_node  # Set the new node to the nearest node
        if 0 <= new_node[0] < image.size[0] and 0 <= new_node[1] < image.size[1]:
            if image.getpixel(new_node) == 255:  # Check if walkable
                tree[new_node] = nearest_node

                distance_to_goal = math.sqrt((new_node[0] - goal[0]) ** 2 + (new_node[1] - goal[1]) ** 2)
                distances_to_goal.append(distance_to_goal)

                if new_node == goal:
                    return new_node, distances_to_goal  # Goal reached

    return None, distances_to_goal  # Goal not reached

# Visualization function
def visualize_rrt(image, tree, goal):
    pixel_access = image.load()

    # Draw the RRT tree
    for node, parent in tree.items():
        if parent:
            draw_line(pixel_access, node, parent, NEON_GREEN)

    # Draw start and end points
    pixel_access[start] = NEON_GREEN
    pixel_access[end] = NEON_GREEN

    image.show()
    image.save("search_result_RRT.png")

# Helper function to draw a line between two points
def draw_line(pixel_access, start, end, color):
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    sx = 1 if start[0] < end[0] else -1
    sy = 1 if start[1] < end[1] else -1
    err = dx - dy

    while (start[0] != end[0] or start[1] != end[1]):
        pixel_access[start] = color
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            start = (start[0] + sx, start[1])
        if e2 < dx:
            err += dx
            start = (start[0], start[1] + sy)

if __name__ == "__main__":
    # Load the image
    im = Image.open("/workspaces/labs-WeffreyJ/searching_map_HW/maps/my_maze2.gif").convert("L")

    # Run the RRT search algorithm
    start_time = time.time()
    goal_node, distances = rrt_search(im, start, end)
    end_time = time.time()
    search_duration = end_time - start_time

    # Visualize the search result on the RGB converted image
    if goal_node:
        im = im.convert('RGB')
        visualize_rrt(im, tree, goal_node)
    else:
        print("Goal not reached.")

    print(f"Search took {search_duration} seconds.")

    # Plot mean and standard deviation for the first 100 iterations
    plt.figure(figsize=(10, 5))
    plt.plot(distances[:100], label='Distance to Goal')
    plt.plot(range(100), [sum(distances[:100])/100]*100, 'r--', label='Mean')
    plt.plot(range(100), [math.sqrt(sum((x - sum(distances[:100])/100)**2 for x in distances[:100])/100)]*100, 'g--', label='Standard Deviation')
    plt.xlabel('Iteration')
    plt.ylabel('Distance')
    plt.title('Distance to Goal over Iterations')
    plt.legend()
    plt.grid(True)
    plt.show()
