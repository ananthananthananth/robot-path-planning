import matplotlib.pyplot as plt
import numpy as np
import json
import os
import pandas as pd
import time
import tracemalloc
import math
import heapq

def calc_obs(g, radius, dim):
    rows, cols = g.shape
    adjcells = []
    for r in range(rows):
        for c in range(cols):
            if g[r, c] == 1:
                for nr in range(max(0, r - int(radius / dim)), min(rows, r + int(radius / dim) + 1)):
                    for nc in range(max(0, c - int(radius / dim)), min(cols, c + int(radius / dim) + 1)):
                        dist = math.hypot((nr - r), (nc - c))
                        if dist <= radius:
                            adjcells.append((nr, nc))
    return adjcells

def ucs(grid, start, goal, radius, reso):
    rows, cols = grid.shape
    open_set = [(0, start)]
    heapq.heapify(open_set)
    closed_set = set()
    parent = dict()

    directions = [
        (0, 1, 1),
        (1, 0, 1),
        (0, -1, 1),
        (-1, 0, 1),
        (1, 1, 1.414),
        (1, -1, 1.414),
        (-1, 1, 1.414),
        (-1, -1, 1.414)
    ]

    danger_cells = calc_obs(grid, radius, reso)

    plt.scatter((start[1] + 0.5), (start[0] + 0.5), color="red", s=600, marker='s', label='Robot')
    plt.scatter((goal[1] + 0.5), (goal[0] + 0.5), color="green", s=600, marker='s', label='Goal')

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current in closed_set:
            continue

        closed_set.add(current)

        if current != start:
            plt.scatter((current[1] + 0.5), (current[0] + 0.5), color="blue", s=600, marker='s', alpha=0.6)

        if current == goal:
            break

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            move_cost = direction[2]

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0], neighbor[1]] == 1:
                    continue
                elif neighbor in danger_cells:
                    plt.scatter((neighbor[1] + 0.5), (neighbor[0] + 0.5), color='black', s=600, marker='s', alpha=0.6)
                    continue

                if neighbor not in closed_set:
                    heapq.heappush(open_set, (current_cost + move_cost, neighbor))
                    parent[neighbor] = current

    if goal not in parent:
        print("Path not found. The goal is unreachable.")
        return [], 0

    start_to_goal = []
    current = goal
    while current != start:
        start_to_goal.append(current)
        current = parent.get(current)

    start_to_goal.append(start)
    start_to_goal.reverse()

    for i in range(len(start_to_goal) - 1):
        curr, next_pos = start_to_goal[i], start_to_goal[i + 1]
        plt.plot([curr[1] + 0.5, next_pos[1] + 0.5],
                 [curr[0] + 0.5, next_pos[0] + 0.5], color="red")

    return start_to_goal, current_cost


def main():
    tracemalloc.start()
    start_time = time.time()
    print("Program execution has been started")

    json_file = os.path.join(os.path.dirname(__file__), "../MapConfig/config.json")

    with open(json_file) as config_env:
        grid_data = json.load(config_env)

    start = grid_data['x_of_start'], grid_data['y_of_start']
    goal = grid_data['x_of_goal'], grid_data['y_of_goal']
    resolution = grid_data['grid_size']
    robot_radius = grid_data['robot_radius']
    excel_file = os.path.join(os.path.dirname(__file__), "../MapConfig/map.xlsx")
    window_size = grid_data['fig']

    data = pd.read_excel(excel_file, header=None).to_numpy()[::-1]

    plt.figure(figsize=(window_size, window_size))
    plt.xlim(1, data.shape[1])
    plt.ylim(1, data.shape[0])
    plt.xticks(np.arange(0, data.shape[1], 1))
    plt.yticks(np.arange(0, data.shape[0], 1))
    plt.grid(True)
    plt.title("Robot Navigation", fontsize=16, fontweight='bold', color="#34495e")

    legend = [
        plt.scatter([], [], color="white", s=200, marker='s', label='Free Space'),
        plt.scatter([], [], color="lightslategrey", s=200, marker='s', label='Obstacle'),
        plt.scatter([], [], color="red", s=200, marker='s', label='Robot'),
        plt.scatter([], [], color="green", s=200, marker='s', label='Goal'),
        plt.scatter([], [], color="black", s=200, marker='s', label='Dangerous Cells')
    ]

    plt.legend(handles=legend, loc='center left', bbox_to_anchor=(0.01, -0.10), ncol=5, fontsize=10)
    plt.tight_layout()

    for iy in range(data.shape[0]):
        for ix in range(data.shape[1]):
            if data[iy, ix] == 1:
                plt.scatter((ix + 0.5), (iy + 0.5), color="lightslategrey", s=1000, marker='s')

    start_to_goal, overall_cost = ucs(data, start, goal, robot_radius, resolution)

    for (row, col) in start_to_goal:
        plt.scatter((col + 0.5), (row + 0.5), color="yellowgreen", s=300, marker='s')

    print("Path found:", start_to_goal)
    print("Total cost:", overall_cost)

    end = time.time()
    print("Program has been ended")
    print("Time used:", end - start_time)

    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()

    plt.show()

if __name__ == "__main__":
    main()
