import numpy as np
from matplotlib import pyplot as plt
import random
from TangentCalculator import find_tangent_points

class Node:
    """A node in the RRT tree.
        - position: The position of the node. (x, y, z)
        - parent: The parent node. (Node)"""

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
    """Circle class for obstacles.
        - position: The position of the center of the circle. (x, y, z)
        - radius: The radius of the circle. (float)
        - robot_radius: The radius of the robot. (float)"""

    def __init__(self, position, radius, robot_radius):
        self.position = position
        self.radius = radius
        self.robot_radius = robot_radius
    
    def point_collision(self, point_pos):
        """Check if a point is inside the circle.
            - point_pos: The position of the point. (x, y, z)"""

        # Calculate the distance between the point and the circle.
        dist = distance(point_pos, self.position)

        # If the distance between the point and the circle is less than the radius, then the point is inside the circle.
        if dist < self.radius + self.robot_radius:
            return True

        return False

    def intersection(self, source_pos, target_pos):
        """Check line-sphere (circle) intersection"""

        # Calculate the direction vector and normalize it by dividing by the distance
        dirn = target_pos - source_pos
        dist = np.linalg.norm(dirn)
        dirn /= dist # normalizDynamicSphereObstaclee

        # Add the robot radius to the circle radius to account for the robot's size
        radius = self.radius + self.robot_radius

        # Calculate the discriminant of the quadratic equation
        a = np.dot(dirn, dirn)
        b = 2 * np.dot(dirn, source_pos - self.position)
        c = np.dot(source_pos - self.position, source_pos - self.position) - radius * radius
        discriminant = b * b - 4 * a * c

        # If the discriminant is negative, there is no intersection
        if discriminant < 0:
            return False

        # Calculate the two intersection points
        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        # If both intersection points are outside the line segment, there is no intersection
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
        - robot_radius: The radius of the robot. (float)
        - plot: A boolean to indicate whether to plot the RRT tree. (bool)"""

    def __init__(self, start_pos, goal_pos, goal_thresh, field_dimensions, max_iterations, max_step_size, n_obstacles, robot_radius, turn_radius, plot):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.goal_thresh = goal_thresh
        self.field_dimensions = field_dimensions
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.n_obstacles = n_obstacles
        self.robot_radius = robot_radius
        self.turn_radius = turn_radius
        self.plot = plot

        self.reached = False

        self.obstacles = []
        self.goal_path = []

        self.generate_start_and_goal_path()

    
    def generate_start_and_goal_path(self):
        """Generate a start and goal path for the RRT algorithm."""

        # Generate a random start position.
        x = np.random.uniform(self.field_dimensions[0][0], 0)
        y = np.random.uniform(self.field_dimensions[1][0], 0)
        z = np.random.uniform(self.field_dimensions[2][0], self.field_dimensions[2][1])
        self.start_pos = np.array([x, y, z])

        self.offset = 2

        # Generate a point with an offset from the start position depending on a random orientation.
        theta = np.random.uniform(0, 0.5*np.pi)
        x = self.start_pos[0] + self.offset * np.cos(theta)
        y = self.start_pos[1] + self.offset * np.sin(theta)
        z = self.start_pos[2]
        new_start_pos = np.array([x, y, z])
        new_start_node = Node(new_start_pos, Node(self.start_pos, None))
        self.nodes = []
        self.nodes.append(new_start_node)

        # Generate a parking position in front of the parking_space.
        self.parking_pos = np.array([self.goal_pos[0], self.goal_pos[1] - self.offset, self.goal_pos[2]])


    def goal_reached(self, node):
        """Check if the goal has been reached.
            - node: The node to check. (Node)"""

        # Calculate the distance between the node and the goal.
        dist = distance(node.position, self.parking_pos)

        # If the distance between the node and the goal is less than the goal threshold, then the goal has been reached, otherwise it has not.
        if dist < self.goal_thresh:
            return True
        else:
            return False


    def pos_with_max_step_size(self, source_pos, target_pos):
        """Calculate the position of the node that is a maximum step size away from the parent node.
            - source_pos: The position of the source node. (x, y, z)
            - target_pos: The position of the target node. (x, y, z)"""

        # Calculate the direction vector between the source and target nodes and normalize it.
        dirn = np.array(target_pos) - np.array(source_pos)
        length = np.linalg.norm(dirn)

        # Scale the direction vector by the maximum step size or the distance between the source and target nodes, whichever is smaller.
        step = (dirn / length) * min(self.max_step_size, length)

        # Calculate the new target position by adding the step to the source position.
        new_target_pos = np.array([source_pos[0]+step[0], source_pos[1]+step[1], 0])

        return new_target_pos

    
    def valid_circle(self, circle):
        """Check if a circle is valid.
            - circle: The circle to check. (Circle)"""

        # Check if the circle is inside the field.
        bottom_check = circle.position[1] - circle.radius > self.field_dimensions[1][0]
        top_check = circle.position[1] + circle.radius < self.field_dimensions[1][1]
        left_check = circle.position[0] - circle.radius > self.field_dimensions[0][0]
        right_check = circle.position[0] + circle.radius < self.field_dimensions[0][1]
        if not (bottom_check and top_check and left_check and right_check):
            return False
        
        # Check if circle is created inside a radius around the start or goal position.
        safety_radius = self.offset + self.robot_radius + 0.5
        start_dist = distance(circle.position, self.start_pos)
        parking_dist = distance(circle.position, self.parking_pos)
        goal_dist = distance(circle.position, self.goal_pos)
        if (start_dist < safety_radius + circle.radius) or (parking_dist < safety_radius + circle.radius) or (goal_dist < safety_radius + circle.radius):
            return False

        # Check if the circle is inside another circle.
        for other_circle in self.obstacles:
            dist = distance(circle.position, other_circle.position)
            if dist < circle.radius + other_circle.radius:
                return False

        return True

    
    def create_circles(self):
        """Create random circles"""

        # Creating n_obstacles random circles
        for i in range(self.n_obstacles):

            # Loop to ensure that the position and radius of the circle are valid
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

                # Create circle object
                circle = Circle([x, y, z], radius, self.robot_radius)

                # If circle is not valid, create a new circle
                if not self.valid_circle(circle):
                    continue
                
                # Add circle to list of obstacles
                self.obstacles.append(circle)
                break

    
    def create_parking_space(self):
        """Create a parking space with spheres"""

        sphere_positions = []
        radius = .5

        for i in range(3):
            sphere_positions.append([self.goal_pos[0]-2*radius, self.goal_pos[1]-radius+i*(2*radius), 0])

        for i in range(3):
            sphere_positions.append([self.goal_pos[0]+2*radius, self.goal_pos[1]-radius+i*(2*radius), 0])

        sphere_positions.append([self.goal_pos[0], self.goal_pos[1] + 3*radius, 0])

        for position in sphere_positions:
            circle = Circle(position, radius, self.robot_radius)
            self.obstacles.append(circle)

    
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
    

    def check_intersection(self, source_pos, target_pos):
        """Check if the line segment between the source and target nodes intersects with an obstacle.
            - source_pos: The position of the source node. (x, y, z)
            - target_pos: The position of the target node. (x, y, z)"""

        # Loop through all the obstacles to check for intersection
        for obstacle in self.obstacles:
            if obstacle.intersection(source_pos, target_pos):
                return True

        return False

    
    def find_closest_reachable_node(self, random_pos):
        """Find the closest node to the random position.
            - random_pos: The position of the random node. (x, y, z)"""
        
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
        
        # Add the goal_node to the end of the path
        self.goal_path.append(Node(self.goal_pos, self.nodes[-1]))

        return self.goal_path


    def create_rrt(self):
        """Create a RRT."""

        # Creating circles for the obstacles in the environment
        self.create_parking_space()
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
    

    def run_rrt(self):
        """Run RRT algorithm"""

        # Create RRT
        self.create_rrt()

        # Check if the goal has been reached and print the result and plot the graph if so
        if self.reached:
            print("Goal reached!")
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
            print("Goal not reached!")
            return False

    
    def rrt_star(self):
        """Create a RRT*."""
        
        # Add the goal node and the parking node to the new path
        new_goal_path = self.goal_path[-2:]

        # Remove the start and goal to make sure that the path will always include the parking node and the node to start driving forward
        goal_path = self.goal_path[1:-1]

        while True:
            
            if(len(goal_path) <= 1):
                break

            destination_node = goal_path[-1]

            for i, source_node in enumerate(goal_path):
                if self.check_intersection(source_node.position, destination_node.position):
                    continue
                else:
                    new_goal_path.insert(0, source_node)
                    goal_path = goal_path[:i+1]
                    break
        
        # Add the start node to the new path
        new_goal_path.insert(0, self.goal_path[0])
        
        self.goal_path = new_goal_path
    

    def add_reeds_shepp_points(self):
        """Add the points from the Reeds-Shepp path to the goal path"""

        shepp_goal_path = [self.goal_path[0]]

        for i in range(1, len(self.goal_path)-1):
            p1 = self.goal_path[i-1].position
            p2 = self.goal_path[i].position
            p3 = self.goal_path[i+1].position
            radius = self.turn_radius

            turn_start, turn_end, turn_center = find_tangent_points(p1, p2, p3, radius)

            shepp_goal_path.append(Node(turn_start, shepp_goal_path[-1]))
            # shepp_goal_path.append(Node(p2, shepp_goal_path[-1]))
            shepp_goal_path.append(Node(turn_end, shepp_goal_path[-1]))
        
        shepp_goal_path.append(Node(self.goal_path[-1].position, shepp_goal_path[-1]))

        self.goal_path = shepp_goal_path
        
    

    def run_rrt_star(self):
        """Run RRT* algorithm"""

        # Create RRT
        self.create_rrt()

        # Check if the goal has been reached and print the result and plot the graph if so
        if self.reached:
            print("Goal reached!")
            if self.plot:
                # Plot graph of nodes and path to goal
                plot_graph = PlotGraph(nodes=self.nodes, 
                                       start_pos=self.start_pos, 
                                       goal_pos=self.goal_pos, 
                                       obstacles=self.obstacles, 
                                       goal_path=self.goal_path, 
                                       field_dimensions=self.field_dimensions)
                plot_graph.create_graph()
            
            self.rrt_star()
            
            if self.plot:
                # Plot graph of nodes and path to goal
                plot_graph = PlotGraph(nodes=self.nodes, 
                                       start_pos=self.start_pos, 
                                       goal_pos=self.goal_pos, 
                                       obstacles=self.obstacles, 
                                       goal_path=self.goal_path, 
                                       field_dimensions=self.field_dimensions)
                plot_graph.create_graph()
            
            self.add_reeds_shepp_points()
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
            print("Goal not reached!")
            return False

            

class PlotGraph:
    """Plot graph of nodes and path to goal.
        - nodes: List of nodes in the RRT.
        - field_dimensions: Dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        - start_pos: The start position of the robot. (x, y, z)
        - goal_pos: The goal position of the robot. (x, y, z)
        - obstacles: List of obstacles in the environment.
        - goal_path: List of nodes from start to goal by following parent nodes."""
        
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
        plt.rcParams["figure.figsize"] = [10.1, 10.1]
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

                # Create a black circle with the radius of the obstacle and add it to the plot
                circle = plt.Circle((obstacle.position[0], obstacle.position[1]), obstacle.radius, color='black')
                plt.gca().add_patch(circle)


    def plot_nodes(self):
        """Plot nodes as dots on graph"""

        # Plot start and goal position
        plt.plot(self.start_pos[0], self.start_pos[1], marker="o", markersize=8, markeredgecolor="red", markerfacecolor="red")
        plt.plot(self.goal_pos[0], self.goal_pos[1], marker="o", markersize=8, markeredgecolor="red", markerfacecolor="red")
        
        # Plot nodes as green dots
        for node in self.nodes:
            x = node.position[0]
            y = node.position[1]
            plt.plot(x, y, marker="o", markersize=2, markerfacecolor="green", markeredgecolor="green")


    def plot_path(self):
        """Plot path to goal"""

        for i in range(len(self.goal_path)-1):

            # Plot line between nodes in the path to goal
            source = self.goal_path[i].position
            target = self.goal_path[i+1].position
            plt.plot(source[0], source[1], marker="o", markersize=4, markerfacecolor="green", markeredgecolor="green")
            plt.plot(target[0], target[1], marker="o", markersize=4, markerfacecolor="green", markeredgecolor="green")
            plt.plot([source[0], target[0]], [source[1], target[1]], 'b-')


if __name__ == "__main__":

    # Set parameters
    start_pos = np.array([-8, -8, 0])
    goal_pos = np.array([8, 8, 0])
    max_iterations = 1000
    max_step_size = 2
    goal_threshold = 0.5
    n_obstacles = 10
    field_dimensions = np.array([(-11, 11), (-11, 11), (0, 0)])
    robot_radius = 0.5
    turn_radius = 1.37
    plot = True


    import timeit

    start = timeit.default_timer()


    #Your statements here

    rrt = RRT(start_pos=start_pos, 
              goal_pos=goal_pos, 
              goal_thresh=goal_threshold, 
              field_dimensions=field_dimensions, 
              max_iterations=max_iterations, 
              max_step_size=max_step_size, 
              n_obstacles=n_obstacles, 
              robot_radius=robot_radius, 
              turn_radius=turn_radius,
              plot=plot)

    rrt.run_rrt_star()

    ###


    stop = timeit.default_timer()

    print('Time: ', stop - start)  

    