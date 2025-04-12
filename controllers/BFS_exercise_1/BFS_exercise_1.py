"""lab_8_solution controller."""

# for this lab, we are going to use a custom Robot class which integrates
# functions to rotate and translate the robot (derived from Lab 7)
from utilities import MyCustomRobot
from collections import deque

# create the custom MyCustomRobot instance and initialize devices
robot = MyCustomRobot(verbose=False)
robot.initialize_devices()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# This map represents the world in which the robot moves.
# Each cell indicates if it is occupied or free.

world_map = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 1, 1, 0, 1, 1, 1, 0],
                [0, 1, 1, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0]
            ]
            
start = (7, 1)  # row, column
goal = (2, 6)    # row, column

def is_valid_position(position):
    """ Returns True if the specified position is within the map and unoccupied. """
    map_size = 8
    if 0 <= position[0] < map_size and 0 <= position[1] < map_size:
        if world_map[position[0]][position[1]] == 0:
            return True
    return False

def get_neighbors(map, position):
    """ Returns the valid neightbors of a cell in the map. """
    north = (position[0]-1, position[1])
    east = (position[0], position[1]+1)
    south = (position[0]+1, position[1])
    west = (position[0], position[1]-1)
    output = []
    for direction in [north, east, south, west]:
        if is_valid_position(direction):
            output.append(direction)
    return output

# START HERE
def path_planning_BFS(map, start, goal):
    """
    This function calculates a path from start to end.
    Input: map of the world, start and end coordinates.
    Output: ordered list of each cell in the path.
    """
    frontier = deque()
    frontier.append(start)
    came_from = dict()
    came_from[start] = None
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break    
        for neighbor in get_neighbors(world_map, current):
            if neighbor not in came_from:
                frontier.append(neighbor)
                came_from[neighbor] = current
    # Build the ordered list
    step = came_from[goal]
    path = [goal]
    while step is not None:
        path.append(step)
        step = came_from[step]
    path.reverse()
    return path
    
path = path_planning_BFS(world_map, start, goal)

def move_from_to(current, next):
    """ Decide in which cardinal direction to move, based on the current and
    next positions. """
    if next[0] < current[0]:
        robot.turn_north()
        robot.move_forward()
    elif next[0] > current[0]:
        robot.turn_south()
        robot.move_forward()
    if next[1] < current[1]:
        robot.turn_west()
        robot.move_forward()
    elif next[1] > current[1]:
        robot.turn_east()
        robot.move_forward()


# Main loop:
while robot.step(timestep) != -1:
    """
    Allow the robot to traverse the calculated path
    You can use the following built-in functions:
     - robot.turn_east()
     - robot.turn_north()
     - robot.turn_west()
     - robot.turn_south()
     - robot.move_forward()
    """
    print("Path: " + str(path))
    current = start
    for next in path[1:]:
        print("Next step: moving from {0} to {1}".format(current, next))
        move_from_to(current, next)
        current = next
    print("Goal reached!")
    break
