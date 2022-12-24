import numpy as np
from matplotlib import pyplot as plt
import random

class Node:
    """A node in the RRT tree."""
    def __init__(self, position, parent):
        self.position = position
        self.parent = parent

    def distance_to_parent(self):
        """Calculate the distance to the parent node using the Euclidean distance formula."""

        error_x = self.position[0] - self.parent.position[0]
        error_y = self.position[1] - self.parent.position[1]
        dist = np.sqrt(error_x**2 + error_y**2)

        return dist

class Circle:
    """Circle class for obstacles"""
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius


def distance(source_pos, target_pos):
    """Calculate the distance between two points using the Euclidean distance formula.
        - source_pos: The position of the source point.
        - target_pos: The position of the target point."""

    error_x = source_pos[0] - target_pos[0]
    error_y = source_pos[1] - target_pos[1]
    dist = np.sqrt(error_x**2 + error_y**2)

    return dist


def angle(source_pos, target_pos):
    """Calculate the angle between two points.
        - source_pos: The position of the source point.
        - target_pos: The position of the target point."""

    dx = target_pos[0] - source_pos[0]
    dy = target_pos[1] - source_pos[1]
    theta = np.arctan(dy/dx)

    return theta


class RRT:
    """A class to represent a Rapidly-Exploring Random Tree (RRT)."""
    def __init__(self, start_pos, goal_pos, goal_thresh, max_iterations, max_step_size, n_obstacles):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.goal_thresh = goal_thresh
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.n_obstacles = n_obstacles
        self.reached = False

        self.obstacles = []

        self.nodes = []
        self.nodes.append(Node(start_pos, None))

        self.goal_path = []


    def goal_reached(self, node):
        """Check if the goal has been reached.
            - node: The node to check.
        """
        # Calculate the distance between the node and the goal.
        dist = distance(node.position, self.goal_pos)

        # If the distance between the node and the goal is less than the goal threshold, then the goal has been reached.
        if dist < self.goal_thresh:
            return True

        return False


    def pos_with_max_step_size(self, source_pos, target_pos):
        """Calculate the position of the node that is a maximum step size away from the parent node.
            - source_pos: The position of the source node.
            - target_pos: The position of the target node.
        """
        # If the distance between the source and target nodes is less than the maximum step size, then the target position is returned.
        dist = distance(source_pos, target_pos)
        if dist <= self.max_step_size:
            return target_pos
        
        # Calculate the angle between the source and target node.
        dx = target_pos[0] - source_pos[0]
        dy = target_pos[1] - source_pos[1]
        theta = np.arctan(dy/dx)

        # Calculate the new position of the node that is a maximum step size away from the source node.
        new_x =  source_pos[0] + self.max_step_size * np.cos(theta)
        new_y =  source_pos[1] + self.max_step_size * np.sin(theta)
        new_target_pos = np.array([new_x, new_y, 0])

        return new_target_pos

    
    def create_circles(self):
        """Create random circles"""

        # Creating n_obstacles random circles
        for i in range(self.n_obstacles):

            # Loop to ensure that the circle is not created on the start or goal position
            while True:
                # Randomly generate radius and position of circle
                radius = random.uniform(0.1, 0.5)
                x = random.uniform(1, 2)
                y = random.uniform(1, 2)

                # Check if circle is created on start or goal position
                if((x-self.start_pos[0])**2 + (y-self.start_pos[1])**2 <= radius**2 or (x-self.goal_pos[0])**2 + (y-self.goal_pos[1])**2 <= radius**2):
                    continue
                
                # Create circle object and add to list of obstacles
                circle = Circle([x, y], radius)
                self.obstacles.append(circle)
                break


    def tangent_angles_of_circle(self, node_pos, circle_obstacle):
        """Calculate the tangent points of a circle.
            - node_pos: The position of the node.
            - circle_obstacle: The circle obstacle."""

        # Define the circle obstacle
        circle_pos = circle_obstacle.position
        r = circle_obstacle.radius
        node_x, node_y = node_pos[0:2]

        d = distance(node_pos, circle_pos)                          # distance between the node and the center of the circle
        theta = angle(node_pos, circle_pos)                         # angle between the node and the center of the circle
        alpha = np.arccos(r / d)                                    # angle between line to the center of the circle and the tangent line

        d1 = theta + alpha                                          # angle of the first tangent line
        d2 = theta - alpha                                          # angle of the second tangent line     

        # Calculate the position of the tangent points
        # tangent_1 = np.array([node_x + r * np.cos(d1), node_y + r * np.sin(d1)])
        # tangent_2 = np.array([node_x + r * np.cos(d2), node_y + r * np.sin(d2)])

        return d1, d2


    def circle_intersection(self, source_pos, target_pos, circle_obstacle):
        """Check if the line segment between the source and target nodes intersects with the circle.
            - source_pos: The position of the source node.
            - target_pos: The position of the target node.
            - circle_obstacle: The circle obstacle."""
        
        # Calculate the tangent points of the circle
        theta_tangent1, theta_tangent2 = self.tangent_angles_of_circle(target_pos, circle_obstacle)

        # d1_x = T1x - source_pos[0]
        # d1_y = T1y - source_pos[1]

        # d2_x = T2x - source_pos[0]
        # d2_y = T2y - source_pos[1]

        # theta_tangent1 = np.arctan(d1_y/d1_x)
        # theta_tangent2 = np.arctan(d2_y/d2_x)

        dtarget_x = target_pos[0] - source_pos[0]
        dtarget_y = target_pos[1] - source_pos[1]
        theta_target = np.arctan(dtarget_y / dtarget_x)

        # Check if the angle between the source and target node is between the two tangent angles, then the line segment intersects with the circle.
        if (theta_tangent1 >= theta_target >= theta_tangent2) or (theta_tangent1 <= theta_target <= theta_tangent2):
            return True
        
        return False


    def create_rrt(self):
        """Create a RRT."""

        # Creating circles for the obstacles in the environment
        self.create_circles()

        # While loop to create the RRT until the goal is reached or the maximum number of iterations is reached
        iter = 0
        while iter <= self.max_iterations:
            iter += 1

            # Randomly generate a position for a new node
            random_pos = np.array([random.uniform(0, 3), random.uniform(0, 3), 0])
            
            min_dist = np.inf

            # Loop through all the nodes to find the closest reachable node to the random position and create a new node
            for node in self.nodes:

                for obstacle in self.obstacles:
                    # Check if obstacle is a circle
                    if type(obstacle) == Circle:
                        # Check if the line segment between the node and the random position intersects with the circle, if so, continue to the next node
                        if self.circle_intersection(node.position, random_pos, obstacle):
                            continue
                                    
                # Calculate the distance between the node and the random position and check if it is the closest node so far
                dist = distance(node.position, random_pos)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = node

            # Calculate the position of the new node considering the maximum step size
            new_pos = self.pos_with_max_step_size(closest_node.position, random_pos)

            # Create a new node and add it to the RRT
            new_node = Node(position=new_pos, parent=closest_node)
            self.nodes.append(new_node)

            # Check if the goal has been reached and if so return self.reached = True
            if self.goal_reached(new_node):
                self.reached = True        
                return self.reached
        
        # If the goal has not been reached after the maximum number of iterations, return self.reached = False
        return self.reached


    def path_to_goal(self):
        """Return list of nodes from start to goal by following parent nodes"""

        # Add the final node to the goal path
        node = self.nodes[-1]
        self.goal_path.append(node)

        # Loop through the parents of the node until the start node is found
        while node.parent is not None:
            # Add the parent node to the start of the list
            node = node.parent
            self.goal_path.insert(0, node)

        return self.goal_path


class PlotGraph:
    """Plot graph of nodes and path to goal"""
    def __init__(self, nodes, start_pos, goal_pos, obstacles, goal_path):
        self.nodes = nodes
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.obstacles = obstacles
        self.goal_path = goal_path


    def create_graph(self):
        """Create graph of nodes and path to goal"""

        # set plot parameters
        plt.rcParams["figure.figsize"] = [10.00, 10.00]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(-0.5, 3.5)
        plt.ylim(-0.5, 3.5)
        plt.grid()

        # plot nodes, path to goal and obstacles
        self.plot_nodes()
        self.plot_path()
        self.plot_obstacles()
        plt.show()
        
        
    def plot_obstacles(self):
        """Plot obstacles"""

        for obstacle in self.obstacles:
            # Check if obstacle is a circle
            if type(obstacle) == Circle:
                circle_1 = plt.Circle((obstacle.position[0], obstacle.position[1]), obstacle.radius, color='black')
                plt.gca().add_patch(circle_1)


    def plot_nodes(self):
        """Plot nodes as dots on graph"""

        # Plot start and goal position
        plt.plot(self.start_pos[0], self.start_pos[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
        plt.plot(self.goal_pos[0], self.goal_pos[1], marker="o", markersize=4, markeredgecolor="red", markerfacecolor="red")
        
        # Plot nodes
        for node in self.nodes:
            x = node.position[0]
            y = node.position[1]
            plt.plot(x, y, marker="o", markersize=2, markerfacecolor="green", markeredgecolor="green")


    def plot_path(self):
        """Plot path to goal"""

        for i in range(len(self.goal_path)-1):
            # Plot line between nodes
            source = self.goal_path[i].position
            target = self.goal_path[i+1].position
            plt.plot([source[0], target[0]], [source[1], target[1]], 'b-')


if __name__ == "__main__":

    # Set parameters
    start_pos=np.array([0, 0, 0])
    goal_pos=np.array([2, 2, 0])
    max_iterations=1000
    max_step_size=0.2
    goal_threshold=0.1
    n_obstacles = 2

    rrt = RRT(start_pos=start_pos, goal_pos=goal_pos, max_iterations=max_iterations, max_step_size=max_step_size, goal_thresh=goal_threshold, n_obstacles=n_obstacles)
    rrt.create_rrt()

    print(rrt.goal_path)

    # If goal not reached, print message
    if not rrt.reached:
        print("Goal not reached...")
    # Else, plot graph
    else:
        print("Goal reached!")
        graph = PlotGraph(rrt.nodes, rrt.start_pos, rrt.goal_pos, rrt.obstacles, rrt.goal_path)
        graph.create_graph()