import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Process
import multiprocessing as mp
import copy

from rrt_dynamic import RRT



class Environment:
    """The environment class that is used to create the environment for the robot to move in.
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max))]
        - obstacles: List of the obstacles that are in the field.
        - robots: List of the robots that are in the field.
        - render: Boolean that determines if the environment should be rendered or not."""

    def __init__(self, field_dimensions, obstacles, robots, render, start_pos):
        self.field_dimensions = field_dimensions
        self.obstacles = obstacles
        self.robots = robots
        self.render = render

        self.obstacle_dict = []

        # Create the environment, add the walls and obstacles
        self.env = gym.make(
            "urdf-env-v0", robots=self.robots, render=self.render
        )
        self.env.reset(pos=start_pos)
        self.env.add_walls()
        self.generate_obstacles()

    
    def generate_obstacles(self):
        """Generates the obstacles in the environment."""

        for obstacle in self.obstacles:
            
            radius = float(obstacle.radius)
            trajectory = obstacle.trajectory

            obs_dict = {
                "type": "sphere",
                "geometry": {"trajectory": trajectory, "radius": radius},
                }
            
            # Create the sphere obstacle
            sphere = DynamicSphereObstacle(name="simpleSphere", content_dict=obs_dict)

            # Add the obstacle to the environment
            self.env.add_obstacle(sphere)


def distance(source_pos, target_pos):
    """Calculate the distance between two points using the Euclidean distance formula.
        - source_pos: The position of the source point.
        - target_pos: The position of the target point."""

    error_x = source_pos[0] - target_pos[0]
    error_y = source_pos[1] - target_pos[1]
    dist = np.sqrt(error_x**2 + error_y**2)

    return dist


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


def move_robot(env, rrt):
    """Moves the robot along the path.
        - env: The environment that the robot is in.
        - rrt: The RRT algorithm that is used to find the path."""
    
    # Disable the plotting of the RRT algorithm to speed up the process
    rrt.plot = False

    # Keep track of the time
    time = 0

    # Create a copy of the RRT algorithm to use for the next path
    current_location = rrt.start_pos
    goal_pos = rrt.goal_pos
    goal_threshold = rrt.goal_threshold
    total_distance = distance(current_location, goal_pos)

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
        if distance(current_location, goal_pos) < goal_threshold:
            return
        
        # Else, run the RRT algorithm again
        rrt.control_step_size_and_threshold(current_location, goal_pos, total_distance)
        print(rrt.goal_threshold, rrt.max_step_size)
        rrt.update_start(current_location)
        rrt.run_rrt(time)
        


def run_point_robot(start_pos, goal_pos, field_dimensions, max_iterations, max_step_size, goal_threshold, n_obstacles, n_dynamic_obstacles, robot_radius, plot=False, render=False):
    """Runs the point robot in the environment.
        - start_pos: The starting position of the robot. (x, y, z)
        - goal_pos: The goal position of the robot. (x, y, z)
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max))]
        - max_iterations: The maximum number of iterations that the RRT algorithm should run for. (int)
        - max_step_size: The maximum step size that the RRT algorithm should take. (float)
        - goal_threshold: The threshold that the RRT algorithm should use to determine if the goal has been reached. (float)
        - n_obstacles: The number of obstacles that should be in the environment. (int)
        - robot_radius: The radius of the robot. (float)
        - plot: Boolean that determines if the RRT algorithm should plot the path or not. (bool)
        - render: Boolean that determines if the environment should be rendered or not. (bool)"""
    
    # Create the robot
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]    
    
    rrt = RRT(start_pos=start_pos, 
              goal_pos=goal_pos, 
              goal_thresh=goal_threshold, 
              field_dimensions=field_dimensions, 
              max_iterations=max_iterations, 
              max_step_size=max_step_size, 
              n_obstacles=n_obstacles,
              n_dynamic_obstacles=n_dynamic_obstacles,
              robot_radius=robot_radius, 
              plot=plot)

    # Run the RRT algorithm and run the algorithm again if the path is not found
    while True:
        rrt.run_rrt(time=0)
        if rrt.reached:
            break

    # Create the environment
    env = Environment(field_dimensions=field_dimensions, 
                      obstacles=rrt.obstacles, 
                      robots=robots, 
                      render=render, 
                      start_pos=start_pos)

    # Move the robot along the path
    move_robot(env, rrt)
    

if __name__ == "__main__":

    start_pos = np.array([-3.0, -3.0, 0.0])
    goal_pos = np.array([3.0, 3.0, 0.0])
    max_iterations = 1000
    max_step_size = 2
    goal_threshold = 0.5
    n_obstacles = 10
    n_dynamic_obstacles = 7
    field_dimensions = np.array([(-3.9, 3.9), (-3.9, 3.9), (0, 0)])
    robot_radius = 0.2
    plot = True
    render = True

    run_point_robot(start_pos=start_pos,
                    goal_pos=goal_pos,
                    field_dimensions=field_dimensions,
                    max_iterations=max_iterations,
                    max_step_size=max_step_size,
                    goal_threshold=goal_threshold,
                    n_obstacles=n_obstacles,
                    n_dynamic_obstacles=n_dynamic_obstacles,
                    robot_radius=robot_radius,
                    plot=plot,
                    render=render)