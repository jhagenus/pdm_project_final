import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
import numpy as np
import matplotlib.pyplot as plt

from rrt_dynamic import RRT


class Environment:
    def __init__(self, field_dimensions, obstacles, robots, render, start_pos):
        self.field_dimensions = field_dimensions
        self.obstacles = obstacles
        self.robots = robots
        self.render = render

        self.obstacle_dict = []

        self.env = gym.make(
            "urdf-env-v0", robots=self.robots, render=self.render
        )
        self.env.reset(pos=start_pos)
        self.env.add_walls()
        self.generate_obstacles()

    
    def generate_obstacles(self):
        for obstacle in self.obstacles:
            if obstacle.dynamic_position == None:
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
            else:
                radius = float(obstacle.radius)
                position = obstacle.dynamic_positon
                
                obs_dict_dynamic = {
                    "type": "sphere",
                    "geometry": {"trajectory": position, "radius": radius},
                    }
                dynamicSphereObst = DynamicSphereObstacle(name="simpleSphere", content_dict=obs_dict_dynamic)
                self.env.add_obstacle(dynamicSphereObst)

def distance(source_pos, target_pos):
    """Calculate the distance between two points using the Euclidean distance formula.
        - source_pos: The position of the source point.
        - target_pos: The position of the target point."""

    error_x = source_pos[0] - target_pos[0]
    error_y = source_pos[1] - target_pos[1]
    dist = np.sqrt(error_x**2 + error_y**2)

    return dist

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
    # Calculate the error between the current location and the desired location
    error_x = desired_location[0] - current_location[0]
    error_y = desired_location[1] - current_location[1]

    # Convert the control signals to actions
    normalize_value = np.sqrt(error_x**2 + error_y**2)

    action = np.array([error_x/normalize_value, error_y/normalize_value, 0])

    return action

def run_point_robot(render=False):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]    

    start_pos = np.array([-3, -3, 0])
    goal_pos = np.array([3, 3, 0])
    max_iterations = 1000
    max_step_size = 1
    goal_threshold = 0.5
    n_obstacles = 3
    n_dynamic_obstacles = 0
    field_dimensions = np.array([(-3.9, 3.9), (-3.9, 3.9), (0, 0)])
    robot_radius = 0.2
    plot = True

    rrt = RRT(start_pos=start_pos, goal_pos=goal_pos, goal_thresh=goal_threshold, field_dimensions=field_dimensions, max_iterations=max_iterations, max_step_size=max_step_size, n_obstacles=n_obstacles, n_dynamic_obstacles=n_dynamic_obstacles, robot_radius=robot_radius, plot=plot, obstacles=None)
    reached = rrt.run_rrt()
    obstacles = rrt.obstacles

    if not reached:
        print("No path found")
        return

    env = Environment(field_dimensions=field_dimensions, obstacles=rrt.obstacles, robots=robots, render=render, start_pos=start_pos)

    update_pos = start_pos
    while True:

        plot = True

        rrt = RRT(start_pos=update_pos, goal_pos=goal_pos, goal_thresh=goal_threshold, field_dimensions=field_dimensions, max_iterations=max_iterations, max_step_size=max_step_size, n_obstacles=n_obstacles, n_dynamic_obstacles=n_dynamic_obstacles, robot_radius=robot_radius, plot=plot, obstacles=obstacles)
        reached = rrt.run_rrt()

        if not reached:
            continue

        # timer van 2 seconde

        
        print("Hallo")
        
        path_to_follow = [node.position for node in rrt.goal_path]

        if len(path_to_follow) == 1:
            break

        deviation_check_x = np.inf

        current_location = path_to_follow[0]
        desired_location = path_to_follow[1]

        while (True):

            action = control_algorithm(current_location, desired_location)

            ob,_,_,_ = env.env.step(action)

            deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )

            if deviation_x > deviation_check_x:

                update_pos[0] = ob['robot_0']['joint_state']['position'][0]
                update_pos[1] = ob['robot_0']['joint_state']['position'][1]
                update_pos[2] = ob['robot_0']['joint_state']['position'][2]

                current_location = update_pos
                break

            deviation_check_x = deviation_x
        
        
        if distance(update_pos, goal_pos) < goal_threshold:
            return

if __name__ == "__main__":
    run_point_robot(render=True)