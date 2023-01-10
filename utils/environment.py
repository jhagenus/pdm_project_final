import numpy as np
import gym
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

import pybullet as p



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
        field_width = self.field_dimensions[0][1] - self.field_dimensions[0][0]
        p.resetDebugVisualizerCamera(cameraDistance=field_width/2, 
                                     cameraYaw=20, 
                                     cameraPitch=-65, 
                                     cameraTargetPosition=[self.field_dimensions[0][1]/10, self.field_dimensions[1][0]/2.5, 0])
        self.generate_outer_walls()
        self.generate_obstacles()

    
    def generate_obstacles(self):
        """Function which generates the obstacle for the gym 
        environment using randomly initialized positions. """
           
        for obstacle in self.obstacles:
            
            # Check if the obstacle is dynamic or not
            if obstacle.trajectory is not None:

                radius = float(obstacle.radius)
                trajectory = obstacle.trajectory

                # Create the obstacle dictionary with the trajectory and radius and add it to the environment
                obs_dict = {
                    "type": "sphere",
                    "geometry": {"trajectory": trajectory, "radius": radius},
                    }
                sphere = DynamicSphereObstacle(name="simpleSphere", content_dict=obs_dict)
                self.env.add_obstacle(sphere)

            else:
                radius = float(obstacle.radius)
                position = [obstacle.position[0],obstacle.position[1], radius]
                position = np.array(position, dtype=float).tolist()

                # Create the obstacle dictionary with the position and radius and add it to the environment
                obs_dict = {
                    "type": "sphere",
                    'movable': obstacle.movable,
                    "geometry": {"position": position, "radius": radius},
                    }
                sphere = SphereObstacle(name="simpleSphere", content_dict=obs_dict)
                self.env.add_obstacle(sphere)                
    

    def generate_outer_walls(self):

        wall_width = 0.2
        wall_height = 0.5
        scale = 1.2
        
        # Create the verticle walls of the environment
        field_height = self.field_dimensions[1][1] - self.field_dimensions[1][0]
        wall_length = scale*field_height + 2 * wall_width
        verticle_wall_dim = np.array([wall_width, wall_length, wall_height])
        verticle_wall_poses = [
                [scale*self.field_dimensions[0][0]-wall_width, wall_width/2, 0],
                [scale*self.field_dimensions[0][1]+wall_width, -wall_width/2, 0]
            ]
        self.env.add_walls(verticle_wall_dim, verticle_wall_poses)

        # Create the horizontal walls of the environment
        field_width = self.field_dimensions[0][1] - self.field_dimensions[0][0]
        wall_length = scale*field_width + 2 * wall_width
        horizontal_wall_dim = np.array([wall_width, wall_length, wall_height])
        horizontal_wall_poses = [
                [-wall_width/2, scale*self.field_dimensions[1][0]-wall_width, 0.5 * np.pi],
                [wall_width/2, scale*self.field_dimensions[1][1]+wall_width, 0.5 * np.pi]
            ]
        self.env.add_walls(horizontal_wall_dim, horizontal_wall_poses)