import sys
from PIL import Image
import copy
import queue
import math
import matplotlib.pyplot as plt

start = (0, 0)
end = (0, 0)
difficulty = ""
G = 0
E = 0
e_list = []

NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

path = []
expanded = {}
frontier = {}

open = queue.PriorityQueue()
came_from = {}
cost_so_far = {}


def search(map):
    def reconstruct_path(came_from, start, end):
        current = end
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def dijkstra(map, heuristic=None):
        global path, expanded, frontier, came_from, cost_so_far

        while not open.empty():
            current, _ = open.get()

            if current == end:
                path = reconstruct_path(came_from, start, end)
                break

            for next in neighbors(current, map):
                new_cost = cost_so_far[current] + 1

                # Add heuristic values if provided
                if heuristic:
                    heuristic_value = heuristic(next, end)
                    new_cost += heuristic_value

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost
                    open.put((next, priority))
                    came_from[next] = current

                expanded[current] = True

    def neighbors(pos, map):
        x, y = pos
        possible_moves = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        valid_moves = [move for move in possible_moves if is_valid(move, map)]
        return valid_moves

    def is_valid(pos, map):
        x, y = pos
        return 0 <= x < len(map) and 0 <= y < len(map[0]) and map[x][y] == 1

    # Call the dijkstra function here
    dijkstra(map, heuristic=euclidean_distance)
    # Add any additional processing or modifications you need for the results


def visualize_search(save_file="searching_map.png"):
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw frontier pixels
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # display and (maybe) save results
    im.show()
    if (save_file != "searching_map.png"):
        im.save(save_file)

    im.close()


def manhattan_distance(pos1, pos2):
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


if __name__ == "__main__":
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print("running " + function_name + " with " + difficulty + " difficulty.")

    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "my_maze.gif":
        start = (0, 0)
        end = (500, 205)
    elif difficulty == "my_maze2.gif":
        start = (0, 0)
        end = (599, 350)
    else:
        assert False, "Incorrect difficulty level provided"

    open = queue.PriorityQueue()
    came_from = {}
    cost_so_far = {}

    G = 1000000000000000000
    E = 1000000000000000000
    open.put((start, 0))
    came_from[start] = None
    cost_so_far[start] = 0

    im = Image.open(difficulty)
    im = im.convert('1')
    search(im.load())
    visualize_search("searching_map.png")
