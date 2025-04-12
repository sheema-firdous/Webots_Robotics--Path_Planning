from utilities import MyCustomRobot
from collections import deque
import heapq
import math

# Instantiate and initialize the custom robot
robot = MyCustomRobot(verbose=True)
robot.initialize_devices()

# Get the simulation timestep
time_step = int(robot.getBasicTimeStep())

# Define a grid-based world map (0 = free space, 1 = obstacle)
grid_map = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 1, 0],
    [0, 1, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0]
]

# Define the start and goal positions (row, col)
start_pos = (7, 1)
goal_pos = (2, 6)

# Manhattan distance heuristic for A* search
def manhattan_heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# A* search algorithm for grid pathfinding
def a_star(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))  # (priority, position)

    came_from = {}  # Tracks best parent node
    g_cost = {start: 0}  # Cost from start to node
    f_cost = {start: manhattan_heuristic(start, goal)}  # Estimated total cost

    while open_list:
        _, current = heapq.heappop(open_list)

        # Goal reached: reconstruct path
        if current == goal:
            final_path = []
            while current in came_from:
                final_path.append(current)
                current = came_from[current]
            final_path.append(start)
            return final_path[::-1]  # Reverse for correct order

        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Neighboring cells
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor

            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
                tentative_g = g_cost[current] + 1

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g
                    f_cost[neighbor] = tentative_g + manhattan_heuristic(neighbor, goal)

                    if neighbor not in [item[1] for item in open_list]:
                        heapq.heappush(open_list, (f_cost[neighbor], neighbor))

    # No valid path found
    return None

# Function to follow a path of grid coordinates
def follow_path(robot, grid, path):
    direction_map = {
        (0, 1): "EAST",
        (0, -1): "WEST",
        (1, 0): "SOUTH",
        (-1, 0): "NORTH"
    }

    current_heading = robot.current_angle

    for index in range(len(path) - 1):
        row1, col1 = path[index]
        row2, col2 = path[index + 1]

        dx, dy = row2 - row1, col2 - col1
        move_direction = direction_map[(dx, dy)]

        # Adjust orientation based on required movement
        if move_direction == "NORTH":
            robot.turn_north()
        elif move_direction == "SOUTH":
            robot.turn_south()
        elif move_direction == "EAST":
            robot.turn_east()
        elif move_direction == "WEST":
            robot.turn_west()

        # Move one cell forward in the grid
        robot.move_forward()

# Run A* search
computed_path = a_star(grid_map, start_pos, goal_pos)

if computed_path:
    print("✅ Path successfully calculated. Beginning traversal...\n")
else:
    print("❌ No path could be found from start to goal.")

# Simulation main loop
while robot.step(time_step) != -1:
    print("🚗 Executing path:", computed_path)
    follow_path(robot, grid_
