import heapq
import time
from PIL import Image
import math

# Initialize start and end points
start = (0, 0)  # Replace with actual starting point coordinates
end = (599, 350)  # Replace with actual ending point coordinates

# Colors for visualization
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)

# Data structures for the algorithm
path = []
expanded = set()
open_set = []
came_from = {}
cost_so_far = {}

# Heuristic functions
def euclidean_heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def manhattan_heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

# Heuristic selection
use_manhattan = True  # Set this to False to use Euclidean heuristic

# A* search algorithm
def a_star_search(image, start, goal, heuristic_func):
    heapq.heappush(open_set, (0, start))
    came_from[start] = None
    cost_so_far[start] = 0

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Neighbors
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < image.size[0] and 0 <= neighbor[1] < image.size[1]:  # Check boundaries
                if image.getpixel(neighbor) == 255:  # Check if walkable
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic_func(goal, neighbor)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current
                        if neighbor not in expanded:
                            expanded.add(neighbor)

    # Reconstruct path
    if goal not in came_from:
        return None, None  # No path found

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path, expanded

# Visualization function
def visualize_search(image, path, expanded):
    pixel_access = image.load()

    # Draw expanded nodes
    for pixel in expanded:
        pixel_access[pixel] = NEON_GREEN

    # Draw the path
    for pixel in path:
        pixel_access[pixel] = PURPLE

    # Draw start and end points
    pixel_access[start] = NEON_GREEN
    pixel_access[end] = NEON_GREEN

    image.show()
    image.save("search_resultA*.png")

if __name__ == "__main__":
    # Load the image
    im = Image.open("/workspaces/labs-WeffreyJ/searching_map_HW/maps/my_maze2.gif").convert("L")

    # Set the heuristic function
    heuristic_func = manhattan_heuristic if use_manhattan else euclidean_heuristic

    # Run the A* search algorithm
    start_time = time.time()
    path, expanded = a_star_search(im, start, end, heuristic_func)
    end_time = time.time()
    search_duration = end_time - start_time

    # Visualize the search result on the RGB converted image
    if path:
        im = im.convert('RGB')
        visualize_search(im, path, expanded)

        # Print the final path cost
        print("Final path cost:", len(path) - 1)  # Subtract 1 to exclude the start node
    else:
        print("No path found from start to end.")

    print(f"Search took {search_duration} seconds.")
    print(f"Expanded nodes: {len(expanded)}")
