import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
import matplotlib.pyplot as plt
from examples.pdm_project_final.obstacles_for_rrt import sphereObst1, obst1Dict

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

def find_nearest_node(goal_pos,rrt,check_value):
    min_dist = np.inf
    nearest_node = goal_pos
    for node in rrt:
        dist = np.linalg.norm(check_value - node)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

def tangent_line_to_circle(location_circle,path,radius_circle,radius_robot):
    # Define the center point and radius of the circle
    # Example values
    (Px, Py) = (location_circle[0], location_circle[1])
    (Cx, Cy) = (path[0], path[1])
    a = radius_circle + radius_robot

    b = np.sqrt((Px - Cx)**2 + (Py - Cy)**2)  # hypot() also works here
    th = np.arccos(a / b)  # angle theta
    d = np.arctan2(Py - Cy, Px - Cx)  # direction angle of point P from C
    d1 = d + th  # direction angle of point T1 from C
    d2 = d - th  # direction angle of point T2 from C

    T1x = Cx +  a * np.cos(d1)
    T1y = Cy * np.sin(d1)
    T2x = Cx * np.cos(d2)
    T2y = Cy * np.sin(d2)

    return T1x,T1y, T2x, T2y


def gen_tree_visual(limit,path,i):
    new_list = []
    for j in range(limit+2):
        if (j > 0 and path[i,j,0] == 0 and path[i,j,1] == 0):
            break
        new_list.append(path[i,j,:])
    return np.array(new_list)


def run_point_robot(max_iterations=500, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",robots=robots, render=render
    )

    start_pos = np.array([0, 0, 0])
    goal_pos = np.array([2, 2, 0])

    new_point = env.reset()
    
    env.add_walls()
    
    if obstacles:
        env.add_obstacle(sphereObst1)
    
    #location obstacle 
    location_obstacle = obst1Dict['geometry']['position']
    obstacle_radius = obst1Dict['geometry']['radius']

    radius_of_robot = 0.2
    
    #Make list where the random values can be stored
    rrt = []
    
    while(True):
        # Sample a random point in the environment
        random_pos = np.concatenate((np.random.uniform(low=-0.5, high=3,size = 2),0),axis = None)

        #in this case check if random point is generated in obstacle
        
        if((random_pos[0]-location_obstacle[0])**2 + (random_pos[1]-location_obstacle[1])**2 <= np.sqrt(obstacle_radius + radius_of_robot)):
            continue

        # Find the nearest node in the RRT
        rrt.append(random_pos)
        if np.linalg.norm(random_pos - goal_pos) < 0.3:
            break
        
    #create array for the tree
    path = np.empty((len(rrt)+2,len(rrt)+2,3))
    path.fill(0)

    #initial value
    path[0,0] = start_pos

    #values for the for loop
    limit = len(rrt)
    check = np.array(rrt[-1])

    print(f'total paths generated = {limit}')

    #check which row to copy for the final path
    row_to_copy = 0 
    
    for k in range(1,limit+2):
        if (k < limit+1):
            dist_history = np.inf
            found_connection = goal_pos
            which_row = 0
            which_column = 0
            for j in range(k):
                for i in range(limit): 
                    if (i > 0 and path[j,i,0] == 0 and path[j,i,1] == 0):
                        break
                    nearest_node = find_nearest_node(goal_pos,rrt,path[j,i,:])

                    # circle obstacle check
                    
                    x1,y1,x2,y2 = tangent_line_to_circle(location_obstacle,path[j,i,:],obstacle_radius,radius_of_robot)

                    tangent_1 = (x1 - path[j,i,0]) / (y1 - path[j,i,1])
                    tangent_2 = (x2 - path[j,i,0]) / (y2 - path[j,i,1])
                    tangent_check = (nearest_node[0] - path[j,i,0]) / (nearest_node[1] - path[j,i,1])

                    if(tangent_1 <= tangent_check <= tangent_2 or tangent_2 <= tangent_check <= tangent_1):
                        continue 
                    # succeeded the check
                    dist = np.linalg.norm(path[j,i,:] - nearest_node)
                    if(dist < dist_history):
                        dist_history = dist
                        found_connection = nearest_node
                        which_row = j
                        which_column = i

                        
            if(found_connection[0] == check[0] and found_connection[1] == check[1]):
                row_to_copy = k

            for z in range(which_column+1):
                path[k,z,:] = path[which_row,z,:]

            path[k,which_column+1,:] = found_connection
            removearray(rrt,found_connection)
        else:
            for t in range(path.shape[1]):
                if (t > 1 and path[row_to_copy,t,0] == 0 and path[row_to_copy,t,1] == 0):
                    path[-1,t,:] = goal_pos
                    break
                path[-1,t,:] = path[row_to_copy,t,:]
            
            print("path to target found")

    
    for i in range(limit + 1): 
        new_array = gen_tree_visual(limit,path,i)
        plt.plot(new_array[:,0],new_array[:,1],color = 'b')

    
    path_to_follow = []
    for j in range(limit+2):
        if (j > 0 and path[-1,j,0] == 0 and path[-1,j,1] == 0):
            break
        path_to_follow.append(path[-1,j,:])

    #print the points to the path
    print("path it will follow")
    print(path_to_follow)

    #make plot of final path
    plot_path = np.array(path_to_follow)
    plt.plot(plot_path[:,0],plot_path[:,1], color = 'r')
    circle_1 =plt.Circle((location_obstacle[0], location_obstacle[1]), obstacle_radius, color='r')
    plt.gca().add_patch(circle_1)
    plt.show()

    for q in range(len(path_to_follow)-1):
        deviation_check_x = np.inf
        while (True):
            current_location = path_to_follow[0]
            desired_location = path_to_follow[1]

            action = control_algorithm(current_location, desired_location)

            ob,_,_,_ = env.step(action)
            deviation_x = abs( (ob['robot_0']['joint_state']['position'][0] - desired_location[0]) / desired_location[0] )

            if deviation_x > deviation_check_x:
                removearray(path_to_follow,current_location)
                path_to_follow[0] = np.array([ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1], 0])
                break
            deviation_check_x = deviation_x

if __name__ == "__main__":
    run_point_robot(render=True)

