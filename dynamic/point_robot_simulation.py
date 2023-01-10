import gym
import numpy as np

from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

import config
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from rrt_dynamic import RRT_Dynamic
from utils.environment import Environment
from utils.plot_graph import PlotGraph


def control_algorithm(current_location, desired_location):
    """Calculate the control signals for the robot to follow the path.
        - current_location: The current location of the robot. [x, y, z]
        - desired_location: The desired location of the robot. [x, y, z]"""

    # Calculate the error between the current location and the desired location
    error_x = desired_location[0] - current_location[0]
    error_y = desired_location[1] - current_location[1]

    # Convert the control signals to actions
    normalize_value = np.sqrt(error_x**2 + error_y**2)
    action = np.array([error_x/normalize_value, error_y/normalize_value, 0])

    return action


# Function to move the robot 1 step in a straight line
def move_robot_1_step(env, rrt, current_location, desired_location):
    """Moves the robot 1 step in a straight line.
        - env: The environment that the robot is in.
        - current_location: The current location of the robot. [x, y, z]
        - desired_location: The desired location of the robot. [x, y, z]"""

    time = 0
    deviation_check_x = np.inf

    while (True):

        # Calculate the action and take a step in the environment
        action = control_algorithm(current_location, desired_location)
        ob,_,_,_ = env.env.step(action)

        # update the time (dt = 0.01)
        time += 0.01

        # If the distance to the desired location is less than the previous distance, the location has been reached
        deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )
        
        if deviation_x > deviation_check_x:

            # Update the current location
            current_location[0] = ob['robot_0']['joint_state']['position'][0]
            current_location[1] = ob['robot_0']['joint_state']['position'][1]
            current_location[2] = ob['robot_0']['joint_state']['position'][2]

            # Update the start position of the RRT algorithm and reset the algorithm
            rrt.update_start(current_location)

            return current_location, time

        deviation_check_x = deviation_x


def move_robot(env, rrt, plot=False):
    """Moves the robot along the path.
        - env: The environment that the robot is in.
        - rrt: The RRT algorithm that is used to find the path."""
    
    # Disable the plotting of the RRT algorithm to speed up the process
    rrt.plot = plot

    # Keep track of the time
    time = 0

    # Create a copy of the RRT algorithm to use for the next path
    current_location = rrt.start_pos
    goal_pos = rrt.goal_pos
    goal_threshold = rrt.goal_threshold
    total_distance = np.linalg.norm(np.subtract(current_location, goal_pos))

    while True:

        # Check if it actually found a path and if not, try again
        if not rrt.reached:
            print("No valid path found, trying again...")
            rrt.update_start(current_location)
            rrt.run_rrt(time)
            continue

        # Get the path to follow and check if the path is valid
        path_to_follow = [node.position for node in rrt.goal_path]

        # Get the current location and the desired location
        current_location = path_to_follow[0]
        desired_location = path_to_follow[1]

        # reset the time
        time = 0
        
        print("Moving the robot...")
        current_location, time_of_movement = move_robot_1_step(env, rrt, current_location, desired_location)
        time += time_of_movement
        
        # Check if the goal has been reached
        if np.linalg.norm(np.subtract(current_location, goal_pos)) < goal_threshold:
            return
        
        # Else, run the RRT algorithm again
        rrt.control_step_size_and_threshold(current_location, goal_pos, total_distance)
        rrt.update_start(current_location)
        rrt.run_rrt(time)
        


def run_point_robot(rrt, plot=False):
    """Runs the point robot in the environment.
        - rrt: The RRT algorithm that has been used to generate the path."""
    
    # Create the robot
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]

    # Run the RRT algorithm and terminate if the goal has not been reached
    rrt.run_rrt(time=0)

    # Create the environment
    env = Environment(field_dimensions = rrt.field_dimensions, 
                      obstacles        = rrt.obstacles, 
                      robots           = robots, 
                      start_pose       = rrt.start_pos)

    # Move the robot along the path
    move_robot(env, rrt, plot)
    

if __name__ == "__main__":

    # Set parameters
    goal_pos                = None
    max_iterations          = 500
    max_step_size           = 2
    goal_threshold          = 0.5
    n_obstacles             = 10
    frac_dynamic_obstacles  = 0.5
    field_dimensions        = np.array([(-4, 4), (-4, 4), (0, 0)])
    robot_width             = 0.4
    plot                    = True  # True: plot the RRT graph at the start
    plot_step               = False  # True: plot the RRT graph after every step

    # Create the RRT
    rrt = RRT_Dynamic(goal_pos               = goal_pos, 
                      goal_threshold         = goal_threshold, 
                      field_dimensions       = field_dimensions, 
                      max_iterations         = max_iterations, 
                      max_step_size          = max_step_size, 
                      n_obstacles            = n_obstacles,
                      frac_dynamic_obstacles = frac_dynamic_obstacles,
                      robot_width            = robot_width,
                      plot                   = plot
                      )
    
    # Run the point robot
    run_point_robot(rrt, plot=plot_step)