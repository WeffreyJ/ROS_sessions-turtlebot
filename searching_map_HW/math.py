import random
import time
from PIL import Image
import math

# Initialize start and end points
start = (0, 0)  # Replace with actual starting point coordinates
end = (500, 205)  # Replace with actual ending point coordinates

# Colors for visualization
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)

# Constants
MAX_ITERATIONS = 10000
MAX_STEP_SIZE = 5

# Data structures for the algorithm
tree = {}

# RRT search algorithm
def rrt_search(image, start, goal):
    tree[start] = None

    for _ in range(MAX_ITERATIONS):
        # Randomly sample a point in the environment
        rand_point = (random.randint(0, image.size[0] - 1), random.randint(0, image.size[1] - 1))

        # Find the nearest node in the tree
        nearest_node = min(tree.keys(), key=lambda x: math.sqrt((x[0] - rand_point[0]) ** 2 + (x[1] - rand_point[1]) ** 2))

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

            if new_node == goal:
                return new_node  # Goal reached

    return None  # Goal not reached

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
    im = Image.open("searching_map_HW/maps/my_maze.gif").convert("L")

    # Run the RRT search algorithm
    start_time = time.time()
    goal_node = rrt_search(im, start, end)
    end_time = time.time()
    search_duration = end_time - start_time

    # Visualize the search result on the RGB converted image
    if goal_node:
        im = im.convert('RGB')
        visualize_rrt(im, tree, goal_node)
    else:
        print("Goal not reached.")

    print(f"Search took {search_duration} seconds.")
