# 🤖 Path Planning Robot with BFS and A* (Webots)

This Webots project implements two classical path planning algorithms — **Breadth-First Search (BFS)** and **A\* (A-Star)** — to guide a robot from a start point to a target while avoiding obstacles.

---

## 🧠 Algorithms Implemented

### 1. 🔄 Breadth-First Search (BFS)

- Explores the search space level-by-level
- Guarantees the shortest path in an unweighted grid
- Slower and less efficient in large search spaces

### 2. ⭐ A* Search

- Uses a heuristic (e.g., Euclidean or Manhattan distance) to guide the search
- Typically faster and more efficient
- Prioritizes promising paths using `f(n) = g(n) + h(n)`

---

## ⚙️ Project Features

- 📍 Robot navigates from a fixed start to a target
- 🚧 Obstacle detection and avoidance built into the map
- 🧭 Real-time decision-making and path execution
- 🔄 Choose between BFS or A* from the controller
- 🗺️ Grid map representation of the environment

---

## 🛠️ Components Used

- **Webots Devices**:
  - `DistanceSensor` (optional for local validation)
  - `GPS`, `InertialUnit` for localization
  - `Motors` for movement
- **Map/Grid**:
  - Discrete grid overlaid on the Webots world
  - Start, goal, and obstacle cells represented as nodes

---

## 📁 Files Overview

| File                  | Description                              |
|-----------------------|------------------------------------------|
| `BFS_exercise_1.py` | Implements BFS path planning             |
| `A_star_exercise_2.py` | Implements A* path planning            |

---

## 🚀 How to Run

1. Open the Webots project world.
2. Ensure the robot has the required devices:
   - `gps`, `inertial unit`, `left wheel motor`, `right wheel motor`
3. Attach the respective controller script:
   - `robot_controller.py` (select algorithm inside)
4. Run the simulation.

---

## 📊 Comparison

| Criteria         | BFS          | A*            |
|------------------|--------------|----------------|
| Optimality       | ✅ Yes        | ✅ Yes         |
| Performance      | ❌ Slower     | ✅ Faster      |
| Memory Usage     | ❌ High       | ✅ Efficient   |
| Heuristic-Based  | ❌ No         | ✅ Yes         |

---

---

## 👩‍💻 Developers Information

Developed by **[Sheema Firdous](https://www.linkedin.com/in/sheema-firdous-67b9b8181/)**  
as part of the **Cognitive Systems and Robotics** module assessment  at **[Sheffield Hallam University](https://www.shu.ac.uk/)**

Supervised by [Dr. Samuele Vinanzi](https://www.linkedin.com/in/samuelevinanzi/)

This project demonstrates the practical application of Path Planning in Cognitive and Autonomous robotics using Webots and Python.
