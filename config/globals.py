from playful_turtlesim.py.world import ROS_world
from playful_turtlesim.py.robot import Robot


# set to false to turn off playful terminal display

PRINT_REPORT=True


# adding a turtle
# which is the "robot" we control

ROBOT = Robot(0,0,0)

# initializing a simulated world
# with 3 turtles swimming randomly
# in it. This simulated world is started
# in on_start.py, and stopped in on_stop.py

ROS_WORLD = ROS_world(3)



