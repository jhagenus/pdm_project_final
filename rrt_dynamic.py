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
    def __init__(self, position, radius, robot_radius, trajectory, trajectory_direction):
        self.position = position
        self.radius = radius
        self.robot_radius = robot_radius
        self.trajectory = trajectory
        self.trajectory_direction = trajectory_direction

    
    def point_collision(self, point_pos):
        """Check if a point is inside the circle.
            - point_pos: The position of the point.
        
        Returns:
            - True if the point is inside the circle.
            - False if the point is outside the circle.
        """
        # Calculate the distance between the point and the circle.
        dist = distance(point_pos, self.position)

        # If the distance between the point and the circle is less than the radius, then the point is inside the circle.
        if dist < self.radius + self.robot_radius:
            return True

        return False

    ### Copied from internet ###
    def intersection(self, source_pos, target_pos):
        """Check line-sphere (circle) intersection"""

        dirn = target_pos - source_pos
        dist = np.linalg.norm(dirn)
        dirn /= dist # normalize

        radius = self.radius + self.robot_radius

        a = np.dot(dirn, dirn)
        b = 2 * np.dot(dirn, source_pos - self.position)
        c = np.dot(source_pos - self.position, source_pos - self.position) - radius * radius

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False

        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        if (t1 < 0 and t2 < 0) or (t1 > dist and t2 > dist):
            return False

        return True


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
    """A class to represent a Rapidly-Exploring Random Tree (RRT).
        - start_pos: The position of the start node. (x, y, z)
        - goal_pos: The position of the goal node. (x, y, z)
        - goal_thresh: The distance threshold for the goal. (float)
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        - max_iterations: The maximum number of iterations to run the RRT algorithm. (int)
        - max_step_size: The maximum step size for the RRT algorithm. (float)
        - n_obstacles: The number of obstacles to create. (int)
        """

    def __init__(self, start_pos, goal_pos, goal_thresh, field_dimensions, max_iterations, max_step_size, n_obstacles, robot_radius, plot, n_dynamic_obstacles, obstacles=None):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.goal_thresh = goal_thresh
        self.field_dimensions = field_dimensions
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.n_obstacles = n_obstacles
        self.robot_radius = robot_radius
        self.plot = plot
        self.obstacles = obstacles

        self.number_dynamic_objects = n_dynamic_obstacles

        self.reached = False

        self.nodes = []
        self.nodes.append(Node(start_pos, None))

        self.goal_path = []
    

    def update_start(self, start_pos):
        """Update the start position and reset the RRT.
            - start_pos: The new start position.
        """
        self.start_pos = start_pos

        # Reset the RRT.
        self.nodes = []
        self.nodes.append(Node(start_pos, None))

        self.reached = False

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

        dirn = np.array(target_pos) - np.array(source_pos)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min (self.max_step_size, length)

        new_target_pos = np.array([source_pos[0]+dirn[0], source_pos[1]+dirn[1], 0])
        return new_target_pos

    
    def valid_circle(self, circle):
        """Check if a circle is valid.
            - circle: The circle to check.
        """

        # Check if the circle is inside the field.
        bottom_check = circle.position[1] - circle.radius > self.field_dimensions[1][0]
        top_check = circle.position[1] + circle.radius < self.field_dimensions[1][1]
        left_check = circle.position[0] - circle.radius > self.field_dimensions[0][0]
        right_check = circle.position[0] + circle.radius < self.field_dimensions[0][1]

        if not (bottom_check and top_check and left_check and right_check):
            return False
        
        # Check if circle is created on start or goal position and if so, create a new circle
        if circle.point_collision(self.start_pos) or circle.point_collision(self.goal_pos):
            return False

        # Check if the circle is inside another circle.
        for other_circle in self.obstacles:
            dist = distance(circle.position, other_circle.position)
            if dist < circle.radius + other_circle.radius:
                return False

        return True

    
    def create_circles(self):
        """Create random circles"""

        self.obstacles = []
        # Count the number of dynamic objects
        count = 0
        
        # Creating n_obstacles random circles
        for i in range(self.n_obstacles):
            # Loop to ensure that the circle is not created on the start or goal position
            while True:
                # Randomly generate radius and position of circle
                field_size = [self.field_dimensions[0][1] - self.field_dimensions[0][0], self.field_dimensions[1][1] - self.field_dimensions[1][0]]
                r_min, r_max = min(field_size)/40, min(field_size)/20
                x_min, x_max = self.field_dimensions[0]
                y_min, y_max = self.field_dimensions[1]

                radius = random.uniform(r_min, r_max)
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                z = 0

                trajectory = None
                trajectory_direction = None

                if(count < self.number_dynamic_objects):
                    self.dynamic = True
                    speed_x = random.uniform(-0.2,0.2)
                    speed_y = random.uniform(-0.2,0.2)
                    x_dynamic = f"{x} + {speed_x} * t"
                    y_dynamic= f"{y} + {speed_y} * t"
                    z_dynamic = f"{radius}"
                    trajectory = [x_dynamic, y_dynamic, z_dynamic]
                    trajectory_direction = [speed_x, speed_y, 0]
                    

                # Create circle object
                circle = Circle(position=[x, y, z], radius=radius, robot_radius=self.robot_radius, trajectory=trajectory, trajectory_direction=trajectory_direction)

                # If circle is not valid, create a new circle
                if not self.valid_circle(circle):
                    print("Invalid circle")
                    continue

                # add 1 to the count
                count += 1
                
                # Add circle to list of obstacles
                self.obstacles.append(circle)
                break

    
    def random_position(self):
        """Generate a random position within the field dimensions"""
        
        # Loop to ensure that the random position is not created inside an obstacle
        while True:
            # Generate random position
            x = random.uniform(self.field_dimensions[0][0], self.field_dimensions[0][1])
            y = random.uniform(self.field_dimensions[1][0], self.field_dimensions[1][1])

            z = 0
            random_pos = np.array([x, y, z])

            # Check if random position is inside an obstacle
            collision = False

            for obstacle in self.obstacles:
                if obstacle.point_collision(random_pos):
                    collision = True
                    break
            
            if not collision:
                return random_pos
    

    def check_collision(self, source_pos, target_pos):
        """Check if the line segment between the source and target nodes intersects with an obstacle.
            - source_pos: The position of the source node.
            - target_pos: The position of the target node.
        """
        dist = distance(source_pos, target_pos)
        d = min(self.field_dimensions[0:2])/300                          # distance between each point on the line segment
        n = int(dist / d)                                                # number of points on the line segment
        
        # Loop through all the points between the source and target nodes to check for collision
        for i in range(n):
            # Calculate the position of the point
            point = source_pos  +  i * d * (target_pos - source_pos) / dist

            # Check if the point is inside an obstacle and return True if so
            for obstacle in self.obstacles:
                if obstacle.point_collision(point):
                    return True

        # Return False if no collision is found
        return False
    

    def check_intersection(self, source_pos, target_pos):

        for obstacle in self.obstacles:
            if obstacle.intersection(source_pos, target_pos):
                return True

        return False

    
    def find_closest_reachable_node(self, random_pos):
        """Find the closest node to the random position.
            - random_pos: The position of the random node."""
        
        min_dist = np.inf
        closest_node = None

        # Loop through all the nodes to find the closest node
        for node in self.nodes:

            # Check if the random position is reachable from the node and if not, continue to the next node
            if self.check_intersection(node.position, random_pos):
                continue
            
            # Calculate the distance between the random position and the node
            dist = distance(random_pos, node.position)

            # Check if the distance is smaller than the minimal distance and if so, update the minimal distance and closest node
            if dist < min_dist:
                min_dist = dist
                closest_node = node

        return closest_node
    

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


    def create_rrt(self,time):
        """Create a RRT."""
        
        # update positions for dynamic obstacles
        if time > 0:
            for obstacle in self.obstacles:
                if obstacle.trajectory is None:
                    continue
                print(obstacle.trajectory_direction)
                obstacle.position[0] = obstacle.trajectory_direction[0] * time + obstacle.position[0]
                obstacle.position[1] = obstacle.trajectory_direction[1] * time + obstacle.position[1]
                obstacle.position[2] = obstacle.trajectory_direction[2] * time + obstacle.position[2]
                

        if self.obstacles is None:
            # Creating circles for the obstacles in the environment
            self.create_circles()

        # While loop to create the RRT until the goal is reached or the maximum number of iterations is reached
        iter = 0
        while iter <= self.max_iterations:
            iter += 1

            # Randomly generate a position for a new node
            random_pos = self.random_position()

            # Find the closest node to the random position that is reachable
            closest_node = self.find_closest_reachable_node(random_pos)

            # If no node is found, continue to the next iteration
            if closest_node is None:
                continue

            # Calculate the position of the new node considering the maximum step size
            new_pos = self.pos_with_max_step_size(closest_node.position, random_pos)

            # Create a new node and add it to the RRT
            new_node = Node(position=new_pos, parent=closest_node)
            self.nodes.append(new_node)

            # Check if the goal has been reached and if so return self.reached = True
            if self.goal_reached(new_node):
                self.path_to_goal()
                self.reached = True        
                return self.reached
        
        # If the goal has not been reached after the maximum number of iterations, return self.reached = False
        return self.reached
    

    def run_rrt(self,time):
        """Run RRT algorithm"""

        # Create RRT
        self.create_rrt(time)

        # Check if the goal has been reached and print the result and plot the graph if so
        if self.reached:
            print("Path to the goal found!")
            if self.plot:
                # Plot graph of nodes and path to goal
                plot_graph = PlotGraph(nodes=self.nodes, 
                                       start_pos=self.start_pos, 
                                       goal_pos=self.goal_pos, 
                                       obstacles=self.obstacles, 
                                       goal_path=self.goal_path, 
                                       field_dimensions=self.field_dimensions)
                plot_graph.create_graph()
            return True

        else:
            print("Path to the goal not found...")
            return False

class PlotGraph:
    """Plot graph of nodes and path to goal"""
    def __init__(self, nodes, field_dimensions, start_pos, goal_pos, obstacles, goal_path):
        self.nodes = nodes
        self.field_dimensions = field_dimensions
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.obstacles = obstacles
        self.goal_path = goal_path


    def create_graph(self):
        """Create graph of nodes and path to goal"""

        # set plot parameters
        plt.rcParams["figure.figsize"] = [10.00, 10.00]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(self.field_dimensions[0][0], self.field_dimensions[0][1])
        plt.ylim(self.field_dimensions[1][0], self.field_dimensions[1][1])

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
    start_pos = np.array([0, 0, 0])
    goal_pos = np.array([2, 2, 0])
    max_iterations = 1000
    max_step_size = 0.3
    goal_threshold = 0.1
    n_obstacles = 3
    n_dynamic_obstacles = 1

    field_dimensions = np.array([(0, 3), (0, 3), (0, 0)])
    robot_radius = 0.2
    plot = True


    import timeit

    start = timeit.default_timer()


    #Your statements here
    rrt = RRT(start_pos=start_pos, goal_pos=goal_pos, goal_thresh=goal_threshold, field_dimensions=field_dimensions, max_iterations=max_iterations, max_step_size=max_step_size, n_obstacles=n_obstacles, robot_radius=robot_radius, plot=plot, n_dynamic_obstacles=n_dynamic_obstacles)

    rrt.run_rrt()

    ###


    stop = timeit.default_timer()

    print('Time: ', stop - start)  

    