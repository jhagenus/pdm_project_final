import numpy as np
from matplotlib import pyplot as plt
import random

class Node:
    def __init__(self, position, parent):
        self.position = position
        self.parent = parent

    def distance_to_parent(self):
        error_x = self.position[0] - self.parent.position[0]
        error_y = self.position[1] - self.parent.position[1]
        dist = np.sqrt(error_x**2 + error_y**2)
        return dist


def distance(source_pos, target_pos):
    error_x = source_pos[0] - target_pos[0]
    error_y = source_pos[1] - target_pos[1]
    dist = np.sqrt(error_x**2 + error_y**2)
    return dist

def goal_reached(node, goal, goal_thresh):
    dist = distance(node.position, goal)
    if dist < goal_thresh:
        return True
    return False

def pos_with_max_step_size(source_pos, target_pos, max_step_size):

    dist = distance(source_pos, target_pos)
    if dist <= max_step_size:
        return target_pos
    
    dx = target_pos[0] - source_pos[0]
    dy = target_pos[1] - source_pos[1]
    theta = np.arctan(dy/dx)

    new_x =  source_pos[0] + max_step_size * np.cos(theta)
    new_y =  source_pos[1] + max_step_size * np.sin(theta)

    new_target_pos = np.array([new_x, new_y, 0])

    return new_target_pos


def tangent_points_of_circle(node_pos, circle_pos, radius_circle):
    # Define the center point and radius of the circle
    # Example values
    (Px, Py) = (circle_pos[0], circle_pos[1])
    (Cx, Cy) = (node_pos[0], node_pos[1])
    a = radius_circle

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

def check_intersection(source_pos, target_pos, circle_pos, radius_circle):
    T1x, T1y, T2x, T2y = tangent_points_of_circle(target_pos, circle_pos, radius_circle)

    dtarget_x = target_pos[0] - source_pos[0]
    dtarget_y = target_pos[1] - source_pos[1]

    d1_x = T1x - source_pos[0]
    d1_y = T1y - source_pos[1]

    d2_x = T2x - source_pos[0]
    d2_y = T2y - source_pos[1]


    theta_tangent1 = np.arctan(d1_y/d1_x)
    theta_tangent2 = np.arctan(d2_y/d2_x)
    theta_target = np.arctan(dtarget_y/dtarget_x)

    if (theta_tangent1 >= theta_target >= theta_tangent2) or (theta_tangent1 <= theta_target <= theta_tangent2):
        return True
    
    return False


def create_rrt(start_pos=np.array([0, 0, 0]), goal_pos=np.array([2, 2, 0]), goal_thresh=0.1, max_iterations=1000, max_step_size=0.2):

    start_node = Node(start_pos, None)
    nodes = [start_node]

    obs_x, obs_y, radius = create_circle(start_pos, goal_pos)
    circle_pos = [obs_x, obs_y]

    iter = 0
    while iter <= max_iterations:
        iter += 1

        random_pos = np.array([random.uniform(0, 3), random.uniform(0, 3), 0])

        min_dist = np.inf
        for node in nodes:
            dist = distance(node.position, random_pos)

            if check_intersection(node.position, random_pos, circle_pos, radius):
                continue

            if dist < min_dist:
                min_dist = dist
                closest_node = node

        new_pos = pos_with_max_step_size(closest_node.position, random_pos, max_step_size)

        new_node = Node(new_pos, closest_node)
        nodes.append(new_node)

        if goal_reached(new_node, goal_pos, goal_thresh=goal_thresh):            
            return nodes
        
    return None


def path_to_goal(final_node):
    path_list = []
    node = final_node
    path_list.append(node)

    while node.parent is not None:
        node = node.parent
        path_list.insert(0, node)

    return path_list


class Circle:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius


class PlotGraph:
    def __init__(self, nodes, start_pos, goal_pos, n_obstacles):
        self.nodes = nodes
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.n_obstacles = n_obstacles
        self.obstacles = []
    
    def create_graph(self):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(0, 5)
        plt.ylim(0, 5)
        plt.grid()

        self.plot_nodes(nodes=nodes, start_pos=start_pos, goal_pos=goal_pos)
        self.plot_path(goal_node)
        self.plot_obstacle()



    def create_circles(self):
        for _ in range(self.n_obstacles):
            while True:
                radius = random.uniform(0.1, 0.5)
                x = random.uniform(1, 2)
                y = random.uniform(1, 2)

                if((x-self.start_pos[0])**2 + (y-self.start_pos[1])**2 <= radius**2 or (x-self.goal_pos[0])**2 + (y-self.goal_pos[1])**2 <= radius**2):
                    continue

                circle = Circle([x, y], radius)
                self.obstacles.append(circle)

        
    def plot_obstacles(self):

        for obstacle in self.obstacles:

            if type(obstacle) == Circle:
                circle_1 = plt.Circle((obstacle.position[0], obstacle.position[1]), obstacle.radius, color='r')
                plt.gca().add_patch(circle_1)


    def plot_nodes(self):
        
        plt.plot(self.start_pos[0], self.start_pos[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
        plt.plot(self.goal_pos[0], self.goal_pos[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
        
        for node in self.nodes:
            x = node.position[0]
            y = node.position[1]
            plt.plot(x, y, marker="o", markersize=2, markerfacecolor="green", markeredgecolor="green")


    def plot_path(self):
        path_list = path_to_goal(self.nodes[-1])

        for i in range(len(path_list)-1):
            source = path_list[i].position
            target = path_list[i+1].position
            
            plt.plot([source[0], target[0]], [source[1], target[1]], 'b-')


if __name__ == "__main__":

    start_pos=np.array([0, 0, 0])
    goal_pos=np.array([2, 2, 0])
    max_iterations=1000
    max_step_size=0.2
    goal_threshold=0.1
    n_obstacles = 2

    nodes= create_rrt(start_pos=start_pos, goal_pos=goal_pos, goal_thresh=goal_threshold, max_iterations=max_iterations, max_step_size=max_step_size)

    if nodes is None:
        print("Goal not reached.")
    else:
        graph = PlotGraph(nodes, start_pos, goal_pos)
        graph.create_circles(n_obstacles)


        plt.show()
