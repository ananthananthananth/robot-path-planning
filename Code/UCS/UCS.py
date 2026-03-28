import matplotlib.pyplot as plt
import numpy as np
import time
import tracemalloc
import math
import heapq

def planner():
    # 1 = obstacle, 0 = free space.
    map = np.array([
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ])

    rp = (2, 2)  # Robot
    gp = (6, 7)  # Goal
    rr = 1  # Robot radius [m]
    reso = 1.0  # Grid resolution

    hurdle = "lightslategrey"
    free_space = "white"
    robot = "red"
    goal = "green"
    explored = "blue"
    path = "yellowgreen"
    danger = "black"

    def adjcells_to_obstacles(grid, radius, dim):
        rows, cols = grid.shape
        adjcells = []

        for ir in range(rows):
            for ic in range(cols):
                if grid[ir, ic] == 1:
                    for i in range(ir - radius, ir + radius + 1):
                        for j in range(ic - radius, ic + radius + 1):
                            if 0 <= i < rows and 0 <= j < cols and math.hypot((i - ir) * dim, (j - ic) * dim) <= radius:
                                adjcells.append((i, j))
        return adjcells

    danger_cells = adjcells_to_obstacles(map, rr, reso)

    def ucs(grid, start, goal):
        rows, cols = grid.shape
        open_set = [(0, start)]  # Priority queue: holds (cost, position)
        heapq.heapify(open_set)
        closed_set = set()  # Keeps track of explored nodes
        parent = dict()  # Tracks the path
        explored_nodes = []

        directions = [
            (0, 1, 1),    # Right
            (1, 0, 1),    # Down
            (0, -1, 1),   # Left
            (-1, 0, 1),   # Up
            (1, 1, 1.414),  # Down-Right (diagonal)
            (1, -1, 1.414), # Down-Left
            (-1, 1, 1.414), # Up-Right
            (-1, -1, 1.414) # Up-Left
        ]
        while open_set:
            current_cost, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)
            explored_nodes.append(current)

            if current != start:
                if current in danger_cells:
                    plt.scatter((current[1] + 0.5) * reso, (current[0] + 0.5) * reso, color=danger, s=600, marker='s', alpha=0.6)
                    plt.pause(0.01)
                else:
                    plt.scatter((current[1] + 0.5) * reso, (current[0] + 0.5) * reso, color=explored, s=600, marker='s', alpha=0.6)
                    plt.pause(0.01)

            if current == goal:
                break

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                move_cost = direction[2]

                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if grid[neighbor[0], neighbor[1]] == 1:
                        continue
                    elif neighbor in danger_cells:
                        plt.scatter(((current[1] + direction[1]) + 0.5) * reso, ((current[0] + direction[0]) + 0.5) * reso, color=danger, s=600, marker='s', alpha=0.6)
                        plt.pause(0.1)
                        continue

                    if neighbor not in closed_set:
                        heapq.heappush(open_set, (current_cost + move_cost, neighbor))
                        parent[neighbor] = current

        if goal not in parent:
            print("Path not found. The goal is unreachable.")
            return [], explored_nodes, current_cost

        start_to_goal = []
        current = goal
        while current != start:
            start_to_goal.append(current)
            current = parent.get(current)

        start_to_goal.append(start)
        start_to_goal.reverse()

        return start_to_goal, explored_nodes, current_cost

    # Grid plotting
    plt.figure(figsize=(7, 7))
    plt.xlim(0, map.shape[1] * reso)
    plt.ylim(0, map.shape[0] * reso)
    plt.xticks(np.arange(0, map.shape[1] * reso, reso))
    plt.yticks(np.arange(0, map.shape[0] * reso, reso))
    plt.grid(True)

    for row in range(map.shape[0]):
        for col in range(map.shape[1]):
            cell_color = hurdle if map[row, col] == 1 else free_space
            plt.scatter((col + 0.5) * reso, (row + 0.5) * reso, color=cell_color, s=1000, marker='s')

    plt.scatter((rp[1] + 0.5) * reso, (rp[0] + 0.5) * reso, color=robot, s=600, marker='s', label='Robot')
    plt.scatter((gp[1] + 0.5) * reso, (gp[0] + 0.5) * reso, color=goal, s=600, marker='s', label='Goal')

    plt.title("Robot Navigator", fontsize=16, fontweight='bold', color="#34495e")

    legend = [
        plt.scatter([], [], color=free_space, s=200, marker='s', label='Free Space'),
        plt.scatter([], [], color=hurdle, s=200, marker='s', label='Obstacle'),
        plt.scatter([], [], color=robot, s=200, marker='s', label='Robot'),
        plt.scatter([], [], color=goal, s=200, marker='s', label='Goal'),
        plt.scatter([], [], color=danger, s=200, marker='s', label='Dangerous Cells')
    ]

    plt.legend(handles=legend, loc='center left', bbox_to_anchor=(0.01, -0.10), ncol=5, fontsize=10)
    plt.tight_layout()

    tracemalloc.start()
    start = time.time()

    start_to_goal, explored_nodes, total_cost = ucs(map, rp, gp)

    for (row, col) in start_to_goal:
        plt.scatter((col + 0.5) * reso, (row + 0.5) * reso, color=path, s=300, marker='s')

    print("Path found:", start_to_goal)
    print("Total cost:", total_cost)

    end = time.time()
    print("Time used:", end - start)

    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()

    plt.show()

if __name__ == "__main__":
    planner()
