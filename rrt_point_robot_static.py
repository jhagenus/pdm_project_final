import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
import numpy as np
import matplotlib.pyplot as plt

from rrt_static import RRT


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
            position = [obstacle.position[0],obstacle.position[1], radius]
            position = np.array(position, dtype=float).tolist()

            # Create the obstacle dictionary with the position and radius and add it to the environment
            obs_dict = {
                "type": "sphere",
                'movable': True,
                "geometry": {"position": position, "radius": radius},
                }
            sphereObst = SphereObstacle(name="simpleSphere", content_dict=obs_dict)
            self.env.add_obstacle(sphereObst)


def removearray(L,arr):
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind],arr):
        ind += 1
    if ind != size:
        L.pop(ind)
    else:
        raise ValueError('array not found in list.')


def control_algorithm(current_location, desired_location):
    """The control algorithm that is used to control the robot.
        - current_location: The current location of the robot. (x, y, z)
        - desired_location: The desired location of the robot. (x, y, z)"""

    # Calculate the error between the current location and the desired location
    error_x = desired_location[0] - current_location[0]
    error_y = desired_location[1] - current_location[1]

    # Convert the control signals to actions
    normalize_value = np.sqrt(error_x**2 + error_y**2)
    action = np.array([error_x/normalize_value, error_y/normalize_value, 0])

    return action


def move_robot(path_to_follow, env):
    """Moves the robot along the path.
        - path_to_follow: The path that the robot should follow. [(x, y, z), (x, y, z), ...]
        - env: The environment that the robot is in."""

    for q in range(len(path_to_follow)-1):
        deviation_check_x = np.inf

        while (True):
            current_location = path_to_follow[0]
            desired_location = path_to_follow[1]

            # Calculate the action that should be taken
            action = control_algorithm(current_location, desired_location)

            # Take the action and get the new state
            ob,_,_,_ = env.step(action)

            # If the distance to the desired location is less than the previous distance, the location has been reached
            deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )
            if deviation_x > deviation_check_x:

                # Remove the current location from the path and set the first location of the path to the current location
                removearray(path_to_follow, current_location)
                path_to_follow[0] = np.array([ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1], 0])
                break

            deviation_check_x = deviation_x
    

def run_point_robot(start_pos, goal_pos, field_dimensions, max_iterations, max_step_size, goal_threshold, n_obstacles, robot_radius, plot=False, render=False):
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
              robot_radius=robot_radius, 
              plot=plot)

    # Run the RRT algorithm and terminate if the goal has not been reached
    reached = rrt.run_rrt_star()
    if not reached:
        return
    
    # Get the path from start to goal
    path_to_follow = [node.position for node in rrt.goal_path]

    # Create the environment
    env = Environment(field_dimensions=field_dimensions, 
                      obstacles=rrt.obstacles, 
                      robots=robots, 
                      render=render, 
                      start_pos=start_pos)

    # Move the robot along the path
    move_robot(path_to_follow, env.env)
        

if __name__ == "__main__":

    start_pos = np.array([-3, -3, 0])
    goal_pos = np.array([3, 3, 0])
    max_iterations = 1000
    max_step_size = 0.3
    goal_threshold = 0.2
    n_obstacles = 5
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
                    robot_radius=robot_radius,
                    plot=plot,
                    render=render)
