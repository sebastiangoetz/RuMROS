# 🧱 Maze Generator for Gazebo / ROS 2

## 📌 Project Goal

This project generates a complete **maze in SDF format** (`.world`), compatible with **Gazebo Harmonic** and **ROS 2 Jazzy**. The maze is fully created in Python, featuring modular wall segments, and can be easily extended with ramps, rooms, goals, or interactive objects.

---

## ✅ Currently Implemented

### ✔️ Maze Generation

* Algorithm: **Iterative Depth-First Search** (DFS) for maze carving
* Guarantees a **fully connected maze** with guaranteed paths between all cells
* Includes **start and goal rooms** (3×3 cells) at opposite corners
* Supports **random interior rooms** with configurable quantity

### ✔️ Pathfinding & Visualization

* Finds the path from start to end using BFS
* Draws the maze with solution path overlayed in `maze_step3_solution.png`

### ✔️ Maze Export to Gazebo World

* Converts maze cells and walls into optimized Gazebo `<include>` models
* Walls combined into large segments (`8m`, `4m`, `2m`, `1m`) for performance
* Maze origin centered at `(0, 0)` with correct wall pose calculation
* Outputs a fully functional `.world` file (`maze_world.world`) for simulation

---

| Step                            | Description                                              | Preview                                                 |
| ------------------------------- | -------------------------------------------------------- | ------------------------------------------------------- |
| **1. Rooms Only Maze**          | Maze after placing start, goal, and random rooms         | ![Rooms Only Maze](./images/maze_step1_rooms_only.png)  |
| **2. After Maze Carving**       | Maze after carving paths between rooms                   | ![After Carving](./images/maze_step2_after_carving.png) |
| **3. Final Maze with Solution** | Maze visualization with shortest path from start to goal | ![Maze with Solution](./images/maze_step3_solution.png) |
| **4. Gazebo World Preview**     | Visualization of the exported maze in Gazebo             | ![Gazebo World](./images/maze_step4_world.png)          |

---

## 📌 Planned Extensions

| Feature               | Status |
| --------------------- | ------ |
| Start/Goal Markings   | ✅ Done |
| Interior Rooms        | ✅ Done |
| Objects (e.g., boxes) | ❌ Open |
| Ramps / Second Floor  | ❌ Open |

---

## 🧠 Key Design Decisions

### Maze Algorithm

* Uses **iterative DFS** for carving paths, ensuring a fully connected maze
* Rooms are explicitly marked as visited areas and carved separately
* Multiple attempts ensure a valid maze with full connectivity

### Wall Optimization

* Detects and merges consecutive wall cells into larger wall segments
* Reduces number of Gazebo model `<include>` elements for better simulation performance
* Supports wall segments of sizes `[8m, 4m, 2m, 1m]`

### Maze Positioning and Export

* Maze centered around `(0, 0)` by offsetting wall positions
* Each wall segment correctly oriented with position and yaw angle
* Generates a valid `.world` file including ground plane and sun light source

---

## 🚀 How to Run

```bash
python3 main.py
```

This will:

* Generate the maze with rooms
* Carve the maze paths
* Find and draw the shortest path solution
* Export the maze as a Gazebo `.world` file named `maze_world.world`
* Save visualization images (`maze_step1_rooms_only.png`, `maze_step2_after_carving.png`, `maze_step3_solution.png`)
