"""robot_utils_demo controller."""

from utilities import MyCustomRobot

# create the custom MyCustomRobot instance and initialize devices
robot = MyCustomRobot()
robot.initialize_devices()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Main loop:
while robot.step(timestep) != -1:
    # Turn the robot to face NORTH
    robot.turn_north()
    # Move forward
    robot.move_forward()
    # Turn the robot to face WEST
    robot.turn_west()
    # Move forward
    robot.move_forward()
    # Turn the robot to face SOUTH
    robot.turn_south()
    # Move forward
    robot.move_forward()
    # Turn the robot to face EAST
    robot.turn_east()
    # Move forward
    robot.move_forward()
    print("Done!")
    break