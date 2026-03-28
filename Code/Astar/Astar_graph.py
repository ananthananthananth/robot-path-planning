import numpy as np
import os
import json
import pandas as pd
import matplotlib.pyplot as plt
import time
import tracemalloc
import math
import heapq

def a_star(grid, start, goal, radius, reso, heuristic_type="manhattan"):
    rows, cols = grid.shape

    # Heuristic
    def heuristic(a, b, heuristic_type):
        if heuristic_type == "manhattan":
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
        elif heuristic_type == "euclidean":
            return math.hypot(a[0] - b[0], a[1] - b[1])  # Euclidean distance
        else:
            raise ValueError("Invalid heuristic type") # Error
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f_cost, node)
    came_from = {}  # To store the path
    g_cost = {start: 0}
    f_cost = {start: heuristic(start, goal, heuristic_type)}  # f(n) = g(n) + h(n)

    moves_with_cost = [(0, 1, 1),  # right
                       (1, 0, 1),  # down 
                       (0, -1, 1),  # left
                       (-1, 0, 1)]  # up

    plt.scatter((start[1] + 0.5), (start[0] + 0.5), color="red", s=600, marker='s', label='Robot') # Initial position
    plt.scatter((goal[1] + 0.5), (goal[0] + 0.5), color="green", s=600, marker='s', label='Goal')  # Final position

    danger_cells = calc_obs(grid, radius, reso)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current != start and current in came_from:
            prev_pos = came_from[current]
            plt.plot([prev_pos[1] + 0.5, current[1] + 0.5], [prev_pos[0] + 0.5, current[0] + 0.5], color="red")

        if current != start:
            plt.scatter((current[1] + 0.5), (current[0] + 0.5), color="blue", s=600, marker='s', alpha=0.6)

        if current == goal:
            break

        for direction in moves_with_cost:
            neighbors = (current[0] + direction[0], current[1] + direction[1])
            cost_of_action = direction[2]

            if 0 <= neighbors[0] < rows and 0 <= neighbors[1] < cols:
                if grid[neighbors[0], neighbors[1]] == 1:  # obstacle
                    continue
                elif neighbors in danger_cells:  # dangerous cells
                    plt.scatter((neighbors[1] + 0.5), (neighbors[0] + 0.5), color='black', s=600, marker='s', alpha=0.6)
                    continue
                else:
                    tentative_g_cost = g_cost[current] + cost_of_action
                    if neighbors not in g_cost or tentative_g_cost < g_cost[neighbors]:
                        came_from[neighbors] = current
                        g_cost[neighbors] = tentative_g_cost
                        f_cost[neighbors] = g_cost[neighbors] + heuristic(neighbors, goal, heuristic_type)
                        heapq.heappush(open_set, (f_cost[neighbors], neighbors))

    if goal not in came_from:
        print("Goal not achievable")
        return [], g_cost, f_cost
    
    path = []
    current_pos = goal
    while current_pos != start:
        path.append(current_pos)
        current_pos = came_from.get(current_pos)

    path.append(start)
    path.reverse()

    overall_cost = g_cost[goal]
    return path, g_cost, overall_cost


def calc_obs(g, radius, dim):
    rows, cols = g.shape
    adjcells = []

    #  speed up distance calculations using numpy
    for r in range(rows):
        for c in range(cols):
            if g[r, c] == 1:
                for nr in range(max(0, r - int(radius / dim)), min(rows, r + int(radius / dim) + 1)):
                    for nc in range(max(0, c - int(radius / dim)), min(cols, c + int(radius / dim) + 1)):
                        dist = math.hypot((nr - r), (nc - c))
                        if dist <= radius:
                            adjcells.append((nr, nc))
    return adjcells


def main(heuristic_type="manhattan"):

    tracemalloc.start() # Memory Used
    start_time = time.time()
    print("Program execution has been started")

    json_file = os.path.join(os.path.dirname(__file__), "../MapConfig/config.json")

    with open(json_file) as config_env:
        grid_data = json.load(config_env)

    # Obstacle positions          
    start = grid_data['x_of_start'], grid_data['y_of_start']  
    goal = grid_data['x_of_goal'], grid_data['y_of_goal']   
    resolution = grid_data['grid_size']  
    robot_radius = grid_data['robot_radius']
    excel_file = os.path.join(os.path.dirname(__file__), "../MapConfig/map.xlsx")
    window_size = grid_data['fig']

    gmap = pd.read_excel(excel_file, header=None)
    data = gmap.to_numpy()
    data = data[::-1]  # Flipping Data

    plt.figure(figsize=(window_size, window_size))
    plt.xlim(1, data.shape[1])
    plt.ylim(1, data.shape[0])
    plt.xticks(np.arange(0, data.shape[1], 1))
    plt.yticks(np.arange(0, data.shape[0], 1))
    plt.grid(True) 

    plt.title("Robot Navigation", fontsize=16, fontweight='bold', color="#34495e")

    # Colors used in grid
    legend = [
        plt.scatter([], [], color="white", s=200, marker='s', label='Free Space'),
        plt.scatter([], [], color="lightslategrey", s=200, marker='s', label='Obstacle'),
        plt.scatter([], [], color="red", s=200, marker='s', label='Robot'),
        plt.scatter([], [], color="green", s=200, marker='s', label='Goal'),
    ]

    plt.legend(handles=legend, loc='center left', bbox_to_anchor=(0.01, -0.10), ncol=5, fontsize=10)
    plt.tight_layout()

    for iy in range(data.shape[0]):
        for ix in range(data.shape[1]):
            if data[iy, ix] == 1:
                plt.scatter((ix + 0.5), (iy + 0.5), color="lightslategrey", s=1000, marker='s')

    path, g_cost, overall_cost = a_star(data, start, goal, robot_radius, resolution, heuristic_type)

    for (row, col) in path:
        plt.scatter((col + 0.5) * 1, (row + 0.5) * 1, color="yellowgreen", s=300, marker='s')

    print("Path taken:", path)
    print("Total cost:", overall_cost)

    end = time.time()
    print("Program has been ended")
    print("Time used", end - start_time,"s")

    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()   

    plt.show()

if __name__ == '__main__':
    main(heuristic_type="euclidean")  # Can be interchanged between "manhattan" or "euclidean"
