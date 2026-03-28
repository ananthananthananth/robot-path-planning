import numpy as np
import json
import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
import time
import tracemalloc
import math

def bfs(grid, start, goal, radius, reso):

    rows, cols = grid.shape
    open_set = deque([(start, 0)])
    closed_set = set() 
    parent = dict()
    
    moves_with_cost =  [(0, 1, 1), #right
                        (1, 0, 1), #down 
                        (0, -1, 1), #left
                        (-1, 0, 1)]  #up 
    
    plt.scatter((start[1] + 0.5), (start[0] + 0.5), color="red", s=600, marker='s', label='Robot')
    plt.scatter((goal[1] + 0.5), (goal[0] + 0.5), color="green", s=600, marker='s', label='Goal') 

    danger_cells = calc_obs(grid, radius, reso)

    while len(open_set) > 0:
            
            current_pos, cost = open_set.popleft()
            if current_pos in parent:
                prev_pos = parent[current_pos]
                plt.plot([prev_pos[1] + 0.5, current_pos[1] + 0.5], [prev_pos[0] + 0.5, current_pos[0] + 0.5], color="red")

            if current_pos != start:
                plt.scatter((current_pos[1] + 0.5), (current_pos[0] + 0.5), color="blue", s=600, marker='s', alpha=0.6)

            if current_pos == goal:
                break

            for direction in moves_with_cost:
                neighbors = (current_pos[0] + direction[0], current_pos[1] + direction[1]) 
                cost_of_action = direction[2]
               
                if 0 <= neighbors[0] < rows and 0 <= neighbors[1] < cols:
                    if grid[neighbors[0], neighbors[1]] == 1: 
                        continue

                    elif neighbors in danger_cells:
                        plt.scatter(((current_pos[1] + direction[1]) + 0.5), ((current_pos[0] + direction[0])+0.5), color='black', s=600, marker='s', alpha=0.6)
                        continue

                    elif neighbors not in closed_set:
                        closed_set.add(neighbors)
                        open_set.append((neighbors, cost + cost_of_action))
                        parent[neighbors] = current_pos
    
    if goal not in parent:
        print("Goal not achievable")
        return [], closed_set, cost
        
    goal_to_start = []
    current_pos = goal
    while current_pos != start: 
        goal_to_start.append(current_pos)
        current_pos = parent.get(current_pos)

    goal_to_start.append(start)
    goal_to_start.reverse()
        
    overall_cost = cost 
    return goal_to_start, closed_set, overall_cost                   


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

def main():

    tracemalloc.start()
    start_time = time.time()
    print("Program execution has been started" )

    json_file = os.path.join(os.path.dirname(__file__), "../MapConfig/config.json")

    with open(json_file) as config_env:
        grid_data = json.load(config_env)

    # Obstacle and free grid positions          
    start = grid_data['x_of_start'],grid_data['y_of_start']  
    goal = grid_data['x_of_goal'],grid_data['y_of_goal']   
    resolution = grid_data['grid_size']  
    robot_radius = grid_data['robot_radius']
    excel_file = os.path.join(os.path.dirname(__file__), "../MapConfig/map.xlsx")
    window_size = grid_data['fig']

    gmap = pd.read_excel(excel_file,header=None)
    data = gmap.to_numpy()
    data = data[::-1]   

    plt.figure(figsize=(window_size ,window_size))
    plt.xlim(1, data.shape[1])
    plt.ylim(1, data.shape[0])
    plt.xticks(np.arange(0,data.shape[1],1))
    plt.yticks(np.arange(0,data.shape[0],1))
    plt.grid(True) 

    plt.title("Robot Navigation", fontsize=16, fontweight='bold', color="#34495e")
    
    #Colors used in grid
    legend = [
        plt.scatter([], [], color="white", s=200, marker='s', label='Free Space'),
        plt.scatter([], [], color="lightslategrey", s=200, marker='s', label='Obstacle'),
        plt.scatter([], [], color="red", s=200, marker='s', label='Robot'),
        plt.scatter([], [], color="green", s=200, marker='s', label='Goal'),
    ]

    plt.legend(handles=legend, loc='center left', bbox_to_anchor=(0.01, -0.10), ncol = 5, fontsize=10)
    plt.tight_layout()

    for iy in range(data.shape[0]):
        for ix in range(data.shape[1]):
            if data[iy, ix] == 1:
                plt.scatter((ix + 0.5), (iy + 0.5), color="lightslategrey", s=1000, marker='s') 

    goal_to_start, closed_set, overall_cost = bfs(data, start, goal, robot_radius, resolution)

    for (row, col) in goal_to_start:
        plt.scatter((col + 0.5) * 1, (row + 0.5) * 1, color="yellowgreen", s=300, marker='s')
    
    print("Path taken:", goal_to_start)
    print("Total cost:", overall_cost)

    end = time.time()
    print("Program has been ended")
    print("Time used", end - start_time)
    
    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()   

    plt.show() 

if __name__ == '__main__':
     main()    

