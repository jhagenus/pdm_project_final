import numpy as np
import gym
from MotionPlanningEnv.sphereObstacle import SphereObstacle



class Environment:
    """The environment class that is used to create the environment for the robot to move in.
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max))]
        - obstacles: List of the obstacles that are in the field.
        - robots: List of the robots that are in the field.
        - render: Boolean that determines if the environment should be rendered or not."""

    def __init__(self, field_dimensions, obstacles, robots, start_pose):
        self.field_dimensions = field_dimensions
        self.obstacles = obstacles

        self.obstacle_dict = []

        # Create the environment, add the walls and obstacles
        self.env = gym.make("urdf-env-v0", robots=robots, render=True)
        self.env.reset(pos=start_pose)
        self.env.add_walls()
        self.generate_obstacles()

    
    def generate_obstacles(self):
        """Function which generates the obstacle for the gym """
        """environment using randomly initialized positions. """
           
        for obstacle in self.obstacles:
            radius = float(obstacle.radius)
            position = [obstacle.position[0],obstacle.position[1], radius]
            position = np.array(position, dtype=float).tolist()

            # Create the obstacle dictionary with the position and radius and add it to the environment
            obs_dict = {
                "type": "sphere",
                'movable': obstacle.movable,
                "geometry": {"position": position, "radius": radius},
                }
            sphereObst = SphereObstacle(name="simpleSphere", content_dict=obs_dict)
            self.env.add_obstacle(sphereObst)