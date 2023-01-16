import numpy as np
import random
import config

from utils.obstacles import Circle
from utils.plot_graph import PlotGraph
from utils.tangent_calculator import find_tangent_points



class Node:
    """A node in the RRT tree.
        - position: The position of the node. np.array(x, y, z)
        - parent: The parent node. (Node)"""

    def __init__(self, position, parent):
        self.position = position
        self.parent = parent



class RRT_Static:
    """A class to represent a Rapidly-Exploring Random Tree (RRT) in a static environment.
        - goal_pos: The position of the goal node. (x, y, z)
        - goal_threshold: The distance threshold for the goal. (float)
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        - max_iterations: The maximum number of iterations to run the RRT algorithm. (int)
        - max_step_size: The maximum step size for the RRT algorithm. (float)
        - n_obstacles: The number of obstacles to create. (int)
        - robot_width: The width of the robot. (float)
        - plot: A boolean to indicate whether to plot the RRT tree before running the simulation. (bool)"""

    def __init__(self, goal_pos, goal_threshold, field_dimensions, max_iterations, max_step_size, n_obstacles, robot_width, turn_radius=0, plot=True, start_pos=None):
        self.goal_pos = goal_pos
        self.goal_threshold = goal_threshold
        self.field_dimensions = field_dimensions
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.n_obstacles = n_obstacles
        self.robot_width = robot_width
        self.turn_radius = turn_radius
        self.plot = plot
        self.start_pos = start_pos

        self.reached = False
        self.obstacles = []
        self.goal_path = []
        self.nodes = []

        # Creating the environment
        self.generate_start_and_goal()
        self.create_parking_space()
        self.create_circles()

    
    def reset(self, goal_pos=None, start_pos=None, obstacles=[]):
        """Reset the RRT tree."""
        self.reached = False
        self.obstacles = obstacles
        self.goal_path = []
        self.nodes = []

        self.goal_pos = goal_pos
        self.start_pos = start_pos

        self.generate_start_and_goal()

    
    def generate_start_and_goal(self):
        """Generate a start and goal path for the RRT algorithm."""

        if self.start_pos is None:
            # Generate a random start position.
            x = np.random.uniform(self.field_dimensions[0][0], 0)
            y = np.random.uniform(self.field_dimensions[1][0], 0)
            z = np.random.uniform(self.field_dimensions[2][0], self.field_dimensions[2][1])
            self.start_pos = np.array([x, y, z])

        # Generate a random goal position.
        if self.goal_pos is None:
            x = np.random.uniform(self.field_dimensions[0][0], self.field_dimensions[0][1])
            y = np.random.uniform(self.field_dimensions[1][0], self.field_dimensions[1][1])
            z = np.random.uniform(self.field_dimensions[2][0], self.field_dimensions[2][1])
            self.goal_pos = np.array([x, y, z])

        self.offset = 1.5 * self.turn_radius
        self.final_goal_pos = self.goal_pos

        if self.offset != 0:
            # Generate a point with an offset from the start position depending on a random orientation.
            self.start_orientation = np.random.uniform(0, 0.5*np.pi)
            x = self.start_pos[0] + self.offset * np.cos(self.start_orientation)
            y = self.start_pos[1] + self.offset * np.sin(self.start_orientation)
            z = self.start_pos[2]
            new_start_pos = np.array([x, y, z])
            new_start_node = Node(new_start_pos, Node(self.start_pos, None))
            self.nodes.append(new_start_node)

            # Generate a parking position in front of the parking_space.
            self.goal_pos = np.array([self.goal_pos[0], self.goal_pos[1] - self.offset - 1, self.goal_pos[2]])
        
        else:
            self.nodes.append(Node(self.start_pos, None))


    def goal_reached(self, node):
        """Check if the goal has been reached.
            - node: The node to check. (Node)"""

        # Calculate the distance between the node and the goal.
        dist = np.linalg.norm(np.subtract(self.goal_pos, node.position))

        # If the distance between the node and the goal is less than the goal threshold, then the goal has been reached, otherwise it has not.
        if dist < self.goal_threshold:
            return True
        else:
            return False


    def pos_with_max_step_size(self, source_pos, target_pos):
        """Calculate the position of the node that is a maximum step size away from the parent node.
            - source_pos: The position of the source node. np.array(x, y, z)
            - target_pos: The position of the target node. np.array(x, y, z)"""

        # Calculate the direction vector between the source and target nodes and calculate the distance
        dirn = np.subtract(target_pos, source_pos)
        dist = np.linalg.norm(dirn)

        # Scale the direction vector by the maximum step size or the distance between the source and target nodes, whichever is smaller.
        step = (dirn / dist) * min(self.max_step_size, dist)

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
        safety_radius = self.offset + (self.robot_width/2) + 0.5
        start_dist = np.linalg.norm(np.subtract(circle.position, self.start_pos))
        goal_dist = np.linalg.norm(np.subtract(circle.position, self.goal_pos))
        final_goal_dist = np.linalg.norm(np.subtract(circle.position, self.final_goal_pos))
        if (start_dist < safety_radius + circle.radius) or (goal_dist < safety_radius + circle.radius) or (final_goal_dist < safety_radius + circle.radius):
            return False

        # Check if the circle is inside another circle.
        for other_circle in self.obstacles:
            dist = np.linalg.norm(np.subtract(circle.position, other_circle.position))
            if dist < circle.radius + other_circle.radius:
                return False

        return True

    
    def create_circles(self):
        """Create random circles"""

        # Initializing the limits of the circles
        field_size = [self.field_dimensions[0][1] - self.field_dimensions[0][0], self.field_dimensions[1][1] - self.field_dimensions[1][0]]
        r_min, r_max = min(field_size)/40, min(field_size)/20
        x_min, x_max = self.field_dimensions[0]
        y_min, y_max = self.field_dimensions[1]

        # Creating n_obstacles random circles
        for i in range(self.n_obstacles):

            # Loop to ensure that the position and radius of the circle are valid
            while True:
                # Randomly generate radius and position of circle
                radius = random.uniform(r_min, r_max)
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                z = 0

                # Create circle object
                circle = Circle([x, y, z], radius, self.robot_width)

                # If circle is not valid, create a new circle
                if not self.valid_circle(circle):
                    continue
                
                # Add circle to list of obstacles and continue to next circle
                self.obstacles.append(circle)
                break

    
    def create_parking_space(self):
        """Create a parking space with spheres"""

        # Initializing the dimensions of the parking space
        parking_height = 2.5 * self.robot_width
        parking_width = 2.5 * self.robot_width
        parking_space_dimensions = [self.final_goal_pos[0]-0.5*parking_width, self.final_goal_pos[0]+0.5*parking_width, 
                                    self.final_goal_pos[1]-(1/3)*parking_height, self.final_goal_pos[1]+(2/3)*parking_height]

        # Initializing the radius of the spheres and the number of spheres
        radius = .2
        n_sides = int(10 * self.robot_width)
        n_back = int(8 * self.robot_width)

        sphere_positions = []

        # Creating spheres on the left and right side of the parking space
        for i in range(n_sides):
            sphere_left = [parking_space_dimensions[0]-radius, parking_space_dimensions[2]+i*(parking_height+radius)/(n_sides-1), radius]
            sphere_right = [parking_space_dimensions[1]+radius, parking_space_dimensions[2]+i*(parking_height+radius)/(n_sides-1), radius]
            sphere_positions.append(sphere_left)
            sphere_positions.append(sphere_right)

        # Creating spheres on the back side of the parking space
        for i in range(n_back):
            sphere_back = [(parking_space_dimensions[0]+0.5*radius)+i*(parking_width-radius)/(n_back-1), parking_space_dimensions[3]+radius, radius]
            sphere_positions.append(sphere_back)

        # Creating the spheres and adding them to the list of obstacles
        for position in sphere_positions:
            circle = Circle(position, radius, self.robot_width, movable=False)
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

            # Check if random position is inside an obstacle and if not, return the position
            collision = False
            for obstacle in self.obstacles:
                if obstacle.point_collision(random_pos):
                    collision = True
                    break
            
            if not collision:
                return random_pos
    

    def check_intersection(self, source_pos, target_pos):
        """Check if the line segment between the source and target nodes intersects with an obstacle.
            - source_pos: The position of the source node. np.array(x, y, z)
            - target_pos: The position of the target node. np.array(x, y, z)"""

        # Loop through all the obstacles to check for intersection
        for obstacle in self.obstacles:
            if obstacle.intersection(source_pos, target_pos):
                return True

        return False

    
    def find_closest_reachable_node(self, random_pos):
        """Find the closest reachable node to the random position.
            - random_pos: The position of the random node. np.array(x, y, z)"""
        
        min_dist = np.inf
        closest_node = None

        # Loop through all the nodes to find the closest node
        for node in self.nodes:

            # Check if the random position is reachable from the node and if not, continue to the next node
            if self.check_intersection(node.position, random_pos):
                continue
            
            # Calculate the distance between the random position and the node
            dist = np.linalg.norm(np.subtract(random_pos, node.position))

            # Check if the distance is smaller than the minimal distance and if so, update the minimal distance and closest node
            if dist < min_dist:
                min_dist = dist
                closest_node = node

        return closest_node
    

    def path_to_goal(self):
        """Return list of nodes from start to goal by following parent nodes"""

        # Add the parking node to the goal path
        node = self.nodes[-1]
        self.goal_path.append(node)

        # Loop through the parents of the node until the start node is found
        while node.parent is not None:
            # Add the parent node to the start of the list
            node = node.parent
            self.goal_path.insert(0, node)
        
        # Add the goal_node to the end of the path
        self.goal_path.append(Node(self.final_goal_pos, self.nodes[-1]))
        self.goal_pos = self.final_goal_pos

        return self.goal_path


    def create_rrt(self):
        """Create a RRT."""

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

    
    def rrt_star(self):
        """Create a RRT* based on the path of the RRT."""
        
        # Add the goal node and the parking node to the new path
        new_goal_path = self.goal_path[-2:]

        # Remove the start and goal to make sure that the path will always include the parking node and the node to start driving forward
        goal_path = self.goal_path[1:-1]

        while True:
            # Check if the goal path is (nearly) empty
            if(len(goal_path) <= 1):
                break
            
            # Define the node that needs to be reached
            destination_node = goal_path[-1]

            # Loop through all nodes in the goal path, starting from the first node
            for i, source_node in enumerate(goal_path):

                # If the destination node is not reachable from the current node, continue to the next node
                if self.check_intersection(source_node.position, destination_node.position):
                    continue

                # If the destination is reachable, add it the current node to the new goal path and remove all nodes after the current node from the goal path
                else:
                    new_goal_path.insert(0, source_node)
                    goal_path = goal_path[:i+1]
                    break
        
        # Add the start node to the new path
        new_goal_path.insert(0, self.goal_path[0])
        
        self.goal_path = new_goal_path
    

    def dubins_path(self):
        """Creates a Dubins path from the RRT* path."""

        # Create a list with the start node
        dubins_goal_path = [self.goal_path[0]]

        # Loop through all nodes in the goal path, starting from the second node
        for i in range(1, len(self.goal_path)-1):
            
            # Find the tangent points between the turning circle and the path at every corner
            p1 = self.goal_path[i-1].position
            p2 = self.goal_path[i].position
            p3 = self.goal_path[i+1].position
            radius = self.turn_radius
            turn_start, turn_end, turn_center = find_tangent_points(p1, p2, p3, radius)

            # Add the turning points to the dubins path
            dubins_goal_path.append(Node(turn_start, dubins_goal_path[-1]))
            dubins_goal_path.append(Node(turn_end, dubins_goal_path[-1]))
        
        # Add the goal node to the dubins path
        dubins_goal_path.append(Node(self.goal_path[-1].position, dubins_goal_path[-1]))

        self.goal_path = dubins_goal_path


    def run_rrt(self, star=True, dubins=True):
        """Run RRT* algorithm"""

        # Create the RRT until the goal is reached
        print("\nLooking for a path to the goal...")
        reached = self.create_rrt()
        while not reached:
            print("Goal not reached, trying again...")
            self.reset(goal_pos=self.goal_pos, start_pos=self.start_pos, obstacles=self.obstacles)
            reached = self.create_rrt()
        print("Goal reached!")

        # Plot graph of nodes and path to goal
        PlotGraph(self, name="Plot of the RRT")
        
        # If star is True, create a RRT* path
        if star:
            print("Creating a shorter path using RRT*...")
            self.rrt_star()
            # Plot graph of nodes and path to goal
            PlotGraph(self, name="Plot of the RRT* path")
            
        # If dubins is True, create a Dubins path
        if dubins:
            print("Creating a path using Dubins curves...")
            self.dubins_path()
            # Plot graph of nodes and path to goal
            PlotGraph(self, name="Plot of the Dubins path")



if __name__ == "__main__":

    # Set parameters
    start_pos           = np.array([-7, -7, 0])
    goal_pos            = np.array([7, 7, 0])
    max_iterations      = 500
    max_step_size       = 2
    goal_threshold      = 0.5
    n_obstacles         = 10
    field_dimensions    = np.array([(-8, 8), (-8, 8), (0, 0)])
    robot_width         = 0.4
    turn_radius         = 0
    plot                = True

    # Create the RRT
    rrt = RRT_Static(goal_pos           = goal_pos, 
                     goal_threshold     = goal_threshold, 
                     field_dimensions   = field_dimensions, 
                     max_iterations     = max_iterations, 
                     max_step_size      = max_step_size, 
                     n_obstacles        = n_obstacles, 
                     robot_width        = robot_width, 
                     turn_radius        = turn_radius,
                     plot               = plot
                     )

    # Run the RRT
    rrt.run_rrt(dubins=False)

    