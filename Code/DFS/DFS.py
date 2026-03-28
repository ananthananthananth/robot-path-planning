# Note: DFS is implemented as a single graph search variant.
# Pure tree DFS (without visited tracking) is not used here as it risks
# infinite loops in grid environments with cycles.

import numpy as np
import json
import os
import pandas as pd
import matplotlib.pyplot as plt
import time
import tracemalloc
import math

def calc_obs(grid, radius, dim):
    rows, cols = grid.shape
    danger_cells = set()
    radius_cells = int(radius / dim)

    obstacle_positions = np.argwhere(grid == 1)
    for r, c in obstacle_positions:
        r_min = max(0, r - radius_cells)
        r_max = min(rows, r + radius_cells + 1)
        c_min = max(0, c - radius_cells)
        c_max = min(cols, c + radius_cells + 1)

        for nr in range(r_min, r_max):
            for nc in range(c_min, c_max):
                if math.hypot(nr - r, nc - c) <= radius:
                    danger_cells.add((nr, nc))
    return danger_cells

def dfs(grid, start, goal, radius, reso):
    rows, cols = grid.shape
    danger_cells = calc_obs(grid, radius, reso)

    def get_neighbors(pos):
        moves = []
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            next_pos = (pos[0] + dx, pos[1] + dy)
            if (0 <= next_pos[0] < rows and
                0 <= next_pos[1] < cols and
                grid[next_pos[0], next_pos[1]] != 1 and
                next_pos not in danger_cells):
             moves.append(next_pos)
        return moves

    stack = [(start, [start])]
    visited = {start: 0}
    best_path = None
    best_cost = float('inf')

    while stack:
        current, path = stack.pop()
        current_cost = len(path) - 1

        if current_cost >= best_cost:
            continue

        if current == goal:
            if current_cost < best_cost:
                best_cost = current_cost
                best_path = path
            continue

        for next_pos in get_neighbors(current):
            new_cost = current_cost + 1
            if next_pos not in visited or new_cost < visited[next_pos]:
                visited[next_pos] = new_cost
                stack.append((next_pos, path + [next_pos]))

    if not best_path:
        return [], visited, 0

    plt.scatter((start[1] + 0.5), (start[0] + 0.5), color="red", s=600, marker='s')
    plt.scatter((goal[1] + 0.5), (goal[0] + 0.5), color="green", s=600, marker='s')

    for i in range(len(best_path) - 1):
        curr, next_pos = best_path[i], best_path[i + 1]
        plt.plot([curr[1] + 0.5, next_pos[1] + 0.5],
                [curr[0] + 0.5, next_pos[0] + 0.5], color="red")
        plt.scatter((next_pos[1] + 0.5), (next_pos[0] + 0.5),
                   color="yellowgreen", s=300, marker='s')

    return best_path, visited, best_cost

def main():
    tracemalloc.start()
    start_time = time.time()

    notebook_dir = os.path.dirname(os.path.abspath(__file__))
    json_file = os.path.join(notebook_dir, "../MapConfig/config.json")
    excel_file = os.path.join(notebook_dir, "../MapConfig/map.xlsx")

    with open(json_file) as config_env:
        grid_data = json.load(config_env)

    start = grid_data['x_of_start'], grid_data['y_of_start']
    goal = grid_data['x_of_goal'], grid_data['y_of_goal']
    resolution = grid_data['grid_size']
    robot_radius = grid_data['robot_radius']

    data = pd.read_excel(excel_file, header=None).to_numpy()[::-1]

    plt.figure(figsize=(grid_data['fig'], grid_data['fig']))
    plt.xlim(1, data.shape[1])
    plt.ylim(1, data.shape[0])
    plt.xticks(np.arange(0, data.shape[1], 1))
    plt.yticks(np.arange(0, data.shape[0], 1))
    plt.grid(True)
    plt.title("Robot Navigation", fontsize=16, fontweight='bold', color="#34495e")

    legend = [plt.scatter([], [], color=c, s=200, marker='s', label=l) for c, l in
             [("white", "Free Space"), ("lightslategrey", "Obstacle"),
              ("red", "Robot"), ("green", "Goal")]]

    plt.legend(handles=legend, loc='center left', bbox_to_anchor=(0.01, -0.10), ncol=5, fontsize=10)
    plt.tight_layout()

    obstacle_positions = np.argwhere(data == 1)
    for r, c in obstacle_positions:
        plt.scatter((c + 0.5), (r + 0.5), color="lightslategrey", s=1000, marker='s')

    path, closed_set, cost = dfs(data, start, goal, robot_radius, resolution)

    print(f"Path taken: {path}\nTotal cost: {cost}")
    print(f"Time used: {time.time() - start_time}")
    print(f"Memory usage - Current: {tracemalloc.get_traced_memory()[0] / 10**6}MB; Peak: {tracemalloc.get_traced_memory()[1] / 10**6}MB")

    tracemalloc.stop()
    plt.show()

if __name__ == '__main__':
    main()
