import sys
import heapq
import time
from PIL import Image
import math

# Initialize start and end points, the image will be loaded to find these points
start = (0, 0)  # This should be the actual starting point coordinates
end = (599, 350)    # This should be the actual ending point coordinates

# Colors for visualization
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)
EXPANDED_COLOR = (153, 255, 153)  # Light green color for expanded nodes

# Data structures for the algorithm
path = []
expanded = {}
frontier = {}
open_set = []
came_from = {}
cost_so_far = {}

# Heuristic functions
def euclidean_heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def manhattan_heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

# Heuristic function for Dijkstra (always returns zero)
def dijkstra_heuristic(a, b):
    return 0

# Search function
def dijkstra_search(map, size, start, end, heuristic_func):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    expanded = set()
    expanded_nodes_counter = 0  # Counter for expanded nodes

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < size[0] and 0 <= neighbor[1] < size[1]:
                if map[neighbor[0], neighbor[1]] == 255 and neighbor not in expanded:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic_func(end, neighbor)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current
                        expanded.add(neighbor)
                        expanded_nodes_counter += 1

    # Reconstruct path from end to start
    path = []
    current = end
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()

    return path, expanded, expanded_nodes_counter,cost_so_far

save_file = "search_result.png"

def visualize_search(im, path, start, end, expanded):
    if im.mode != 'RGB':
        im = im.convert('RGB')
    pixel_access = im.load()

    # Draw the expanded nodes
    for pixel in expanded:
        pixel_access[pixel[0], pixel[1]] = EXPANDED_COLOR

    # Draw the path if it exists
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # Draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    im.show()
    if save_file:
        im.save(save_file)
    return im

cost_so_far = {}  # Global declaration

if __name__ == "__main__":
    # Set the start and end according to the maze
    start = (0, 0)  # Replace with actual coordinates
    end = (599, 350)  # Replace with actual coordinates

    difficulty = "/workspaces/labs-WeffreyJ/searching_map_HW/maps/my_maze2.gif"
    im = Image.open(difficulty).convert('1')

    # Start timing the search
    start_time = time.time()

    im_size = im.size
    
    # Run the search algorithm
    path, expanded, expanded_nodes_counter, cost_so_far = dijkstra_search(im.load(), im_size, start, end, dijkstra_heuristic)
    
    # End timing the search
    end_time = time.time()
    
    # Calculate the duration of the search
    search_duration = end_time - start_time

    if path:
        # Visualize the search result
        im = im.convert('RGB')
        visualize_search(im, path, start, end, expanded)

        # Save the visualized search result
        save_file = "search_result.png"
        im.save(save_file)

        # Output the results
        print(f"Search took {search_duration:.4f} seconds.")
        print(f"Expanded nodes: {expanded_nodes_counter}")
        print(f"Final path cost: {cost_so_far[end]}")
    else:
        print("No path found from start to end.")
