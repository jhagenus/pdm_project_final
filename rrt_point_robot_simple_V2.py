import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
import matplotlib.pyplot as plt

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


def run_point_robot(max_iterations=500, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",robots=robots, render=render
    )

    start_pos = np.array([0, 0, 0])
    goal_pos = np.array([1, 1, 0])

    new_point = env.reset()
    
    env.add_walls()
    if obstacles:
        from examples.pdm_project_final.obstacles_for_rrt import (
            sphereObst1,
        )
        env.add_obstacle(sphereObst1)
        
    rrt = [start_pos]
    
    while(True):
        # Sample a random point in the environment
        random_pos = np.concatenate((np.random.uniform(low=-2, high=2,size = 2),0),axis = None)
        # Find the nearest node in the RRT
        rrt.append(random_pos)
        if np.linalg.norm(random_pos - goal_pos) < 0.1:
            print('path is found')
            break
        
    path = [goal_pos]

    while(True):
        check_value = np.array(path[-1])
        if(check_value[0] == start_pos[0] and check_value[1] == start_pos[1]):
            print("origin_found")
            break
        min_dist = np.inf
        nearest_node = start_pos
        for node in rrt:
            dist = np.linalg.norm(check_value - node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        path.append(nearest_node)
        removearray(rrt,nearest_node)
        
    #reverse the path
    path = path[::-1]

    #plot the path
    plot = np.array(path)
    plt.plot(plot[:,0],plot[:,1])
    plt.show()

    deviation_check_x = np.inf

    while (len(path) > 1):
        current_location = path[0]
        desired_location = path[1]

        action = control_algorithm(current_location, desired_location)

        ob,_,_,_ = env.step(action)
        
        deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )

        if deviation_x > deviation_check_x:
            removearray(path,current_location)
            path[0] = np.array([ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1], 0])
        
        deviation_check_x = deviation_x

if __name__ == "__main__":
    run_point_robot(render=True)


