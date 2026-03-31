# 🤖 Robot Path Planning — Search Algorithm Comparison

Implementing and comparing classic AI search algorithms (BFS, DFS, UCS, and A*) for mobile robot navigation on a 2D grid environment. Both **tree search** and **graph search** paradigms are implemented and benchmarked against each other.

---

## 📌 Project Overview

A mobile robot must navigate from a start position **A** to a goal position **B** on a 13×13 grid containing 4 impassable obstacle cells. The project evaluates which search algorithm finds the best path most efficiently, measuring execution time, memory usage, and path cost across 7 algorithm variants (4 algorithms, with DFS implemented as a single graph search variant).

---

## 🗺️ Environment

| Property | Details |
| --- | --- |
| Grid size | 13 × 13 |
| Obstacles | 4 impassable obstacle cells |
| Start position | (2, 3) |
| Goal position | (9, 9) |
| Robot shape | Circular base |
| Motion model | BFS, DFS, A*: Up, Down, Left, Right (cost = 1); UCS: 8 directions including diagonals (diagonal cost = 1.414) |
| Representation | Binary matrix (0 = free, 1 = obstacle) |

The shared map and configuration used by all algorithms is in `Code/MapConfig/`.

---

## 🧠 Algorithms Implemented

| Algorithm | Variant | Description |
| --- | --- | --- |
| **BFS** | Tree & Graph | Breadth-First Search — explores level by level; guarantees shortest path in uniform cost environments |
| **DFS** | Graph only | Depth-First Search — explores deep branches first; memory-efficient. Tree variant omitted as it risks infinite loops in grid environments with cycles |
| **UCS** | Tree & Graph | Uniform Cost Search — expands lowest-cost node first; optimal. Supports diagonal movement |
| **A\*** | Tree & Graph | A-Star — uses Euclidean or Manhattan heuristic to guide search; optimal and efficient |

> **Tree search** does not track visited nodes, while **graph search** maintains a closed set to avoid revisiting nodes — trading memory for speed.

---

## 📊 Benchmark Results

All results measured on the shared 13×13 grid from start (2,3) to goal (9,9).

| Algorithm | Heuristic | Time (s) | Memory (MB) | Path Cost |
| --- | --- | --- | --- | --- |
| T-A* | Manhattan | 0.80 | 10.6 | 13 |
| T-A* | Euclidean | 0.81 | 11.0 | 13 |
| G-A* | Manhattan | 0.80 | 10.6 | 13 |
| G-A* | Euclidean | 0.83 | 11.0 | 13 |
| T-BFS | — | 0.89 | 12.3 | 13 |
| G-BFS | — | 0.88 | 12.3 | 13 |
| DFS | — | 0.66 | 9.3 | 13 |
| T-UCS | — | 0.82 | 11.0 | 9.484 |
| G-UCS | — | 0.84 | 11.5 | 9.484 |

> UCS achieves a lower path cost (9.484) than other algorithms by using diagonal movement, which allows shorter routes at a cost of 1.414 per diagonal step.

---

## 📁 Repository Structure

```
robot-path-planning/
├── Code/
│   ├── Astar/
│   │   ├── AStar_Graph.py      # A* graph search
│   │   └── AStar_Tree.py       # A* tree search
│   ├── BFS/
│   │   ├── BFS_Graph.py        # BFS graph search
│   │   └── BFS_Tree.py         # BFS tree search
│   ├── DFS/
│   │   └── DFS.py              # DFS graph search 
│   ├── UCS/
│   │   ├── UCS_Graph.py        # UCS graph search
│   │   └── UCS_Tree.py         # UCS tree search
│   └── MapConfig/
│       ├── map.xlsx            # Shared 13×13 grid map
│       └── config.json         # Environment configuration
└── README.md
```

---

## 🛠️ Tech Stack

* **Python 3**
* **Libraries:** NumPy, Pandas, Matplotlib, openpyxl, heapq
* **Platform:** VS Code / local Python environment

---

## 🔧 Code Notes

The A* implementation was written from scratch as the primary algorithm for this project. The remaining algorithm files (BFS, DFS, UCS) were reviewed post-submission, with some files rewritten and others replaced to improve correctness and performance.

---

## 🚀 How to Run

1. Clone the repository:

   ```
   git clone https://github.com/ananthananthananth/robot-path-planning.git
   ```

2. Install dependencies:

   ```
   pip install numpy matplotlib pandas openpyxl
   ```

3. Navigate to any algorithm folder and run the relevant file:

   ```
   cd Code/Astar
   python AStar_Graph.py   # or AStar_Tree.py

   cd Code/BFS
   python BFS_Graph.py     # or BFS_Tree.py

   cd Code/DFS
   python DFS.py

   cd Code/UCS
   python UCS_Graph.py     # or UCS_Tree.py
   ```

> **Note:** A* heuristic can be switched between `"manhattan"` and `"euclidean"` by changing the `heuristic_type` argument in the `main()` call at the bottom of each A* file.
