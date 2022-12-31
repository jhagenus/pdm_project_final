import gym
from MotionPlanningEnv.sphereObstacle import SphereObstacle
import numpy as np
import matplotlib.pyplot as plt
from urdfenvs.robots.prius import Prius
import math

from rrt_static import RRT
from rrt_point_robot_V6 import removearray


class Environment:
    def __init__(self, field_dimensions, obstacles, robots, render, start_pos):
        self.field_dimensions = field_dimensions
        self.obstacles = obstacles
        self.robots = robots
        self.render = render

        self.obstacle_dict = []

        self.env = gym.make(
            "urdf-env-v0",robots=self.robots, render=self.render
        )
        self.env.reset(pos=start_pos)
        self.env.add_walls()
        self.generate_obstacles()

    def generate_obstacles(self):
        for obstacle in self.obstacles:
            radius = float(obstacle.radius)
            position = obstacle.position
            position = [position[0],position[1], radius]
            position = np.array(position, dtype=float).tolist()

            obs_dict = {
                "type": "sphere",
                'movable': True,
                "geometry": {"position": position, "radius": radius},
                }
            sphereObst = SphereObstacle(name="simpleSphere", content_dict=obs_dict)
            self.env.add_obstacle(sphereObst)



def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
 
    start_pos = np.array([-3, -3, 0])
    goal_pos = np.array([3, 3, 0])
    max_iterations = 1000
    max_step_size = 0.3
    goal_threshold = 0.5

    n_obstacles = 1
    field_dimensions = np.array([(-3.9, 3.9), (-3.9, 3.9), (0, 0)])
    robot_radius = 0.3
    plot = True

    rrt = RRT(start_pos=start_pos, goal_pos=goal_pos, goal_thresh=goal_threshold, field_dimensions=field_dimensions, max_iterations=max_iterations, max_step_size=max_step_size, n_obstacles=n_obstacles, robot_radius=robot_radius, plot=plot)
    reached = rrt.run_rrt()

    if not reached:
        return

    path_to_follow = [node.position for node in rrt.goal_path]

    Env = Environment(field_dimensions=field_dimensions, obstacles=rrt.obstacles, robots=robots, render=render, start_pos=start_pos)

    for q in range(len(path_to_follow)-1):
        deviation_check_x = np.inf
        while (True):
            current_location = path_to_follow[0]
            desired_location = path_to_follow[1]

            action = np.array([2.,math.sin(q)]) #control algorithm
             
            ob,_,_,_ = Env.env.step(action)
            deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )

            if deviation_x > deviation_check_x:
                removearray(path_to_follow,current_location)
                path_to_follow[0] = np.array([ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1], 0])
                break
            deviation_check_x = deviation_x

    

if __name__ == "__main__":
    robots = [
        Prius(mode="vel"),
    ]


    run_prius(render=True)