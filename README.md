# 🤖 Robot Path Planning — Search Algorithm Comparison

Implementing and comparing classic AI search algorithms (BFS, DFS, UCS, and A*) for mobile robot navigation on a 2D grid environment. Both **tree search** and **graph search** paradigms are implemented and benchmarked against each other.

---

## 📌 Project Overview

A mobile robot must navigate from a start position **A** to a goal position **B** on a 12×12 grid containing 2 impassable obstacle regions. The project evaluates which search algorithm finds the best path most efficiently, measuring execution time, memory usage, path cost, and path length across 8 algorithm variants (4 algorithms × 2 search paradigms).

---

## 🗺️ Environment

| Property | Details |
|---|---|
| Grid size | 12 × 12 |
| Obstacles | 2 impassable obstacle regions |
| Robot shape | Circular base |
| Motion model | Up, Down, Left, Right (cost = 1 per move) |
| Representation | Binary matrix (0 = free, 1 = obstacle) |

The shared map and configuration used by all algorithms is in `Code/MapConfig/`.

---

## 🧠 Algorithms Implemented

Each of the following is implemented under both **tree search** (Set 1) and **graph search** (Set 2) paradigms:

| Algorithm | Description |
|---|---|
| **BFS** | Breadth-First Search — explores level by level; optimal for uniform costs |
| **DFS** | Depth-First Search — explores deep branches first; memory-efficient but not optimal |
| **UCS** | Uniform Cost Search — expands lowest-cost node first; optimal |
| **A\*** | A-Star — uses heuristic (Euclidean/Manhattan distance) to guide search; optimal and efficient |

> **Tree search** re-explores nodes (no visited tracking), while **graph search** maintains a closed set to avoid revisiting nodes — trading memory for speed.

---

## 📁 Repository Structure

```
robot-path-planning/
├── Code/
│   ├── Astar/
│   │   ├── Astar_graph.py      # A* graph search
│   │   └── Astar_tree.py       # A* tree search
│   ├── BFS/
│   │   ├── BFS_Graph.py        # BFS graph search
│   │   └── BFS_Tree.py         # BFS tree search
│   ├── DFS/
│   │   └── DFS.py              # DFS implementation
│   ├── UCS/
│   │   ├── UCS.py              # UCS graph search
│   │   └── UCS_Tree.py         # UCS tree search
│   └── MapConfig/
│       ├── map.xlsx            # Shared 12×12 grid map
│       └── config.json         # Environment configuration
└── README.md
```

---

## 📊 Performance Metrics Tracked

Each algorithm records:
- **Path found** — sequence of coordinates from start to goal
- **Path cost** — total cost of the path
- **Execution time** — in seconds
- **Memory/storage** — nodes stored during search

---

## 🛠️ Tech Stack

- **Python 3**
- **Libraries:** NumPy, Pandas, Matplotlib, openpyxl
- **Platform:** VS Code / local Python environment

---

## 🚀 How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/ananthananthananth/robot-path-planning.git
   ```
2. Install dependencies:
   ```bash
   pip install numpy matplotlib pandas openpyxl
   ```
3. Navigate to any algorithm folder and run the relevant file:
   ```bash
   cd Code/Astar
   python Astar_graph.py   # or Astar_tree.py

   cd Code/BFS
   python BFS_Graph.py     # or BFS_Tree.py

   cd Code/DFS
   python DFS.py

   cd Code/UCS
   python UCS.py           # or UCS_Tree.py
   ```

---
