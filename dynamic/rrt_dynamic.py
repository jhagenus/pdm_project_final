import numpy as np
from matplotlib import pyplot as plt
import random
import config

from utils.obstacles import Circle
from utils.plot_graph import PlotGraph



class Node:
    """A node in the RRT tree.
        - position: The position of the node. np.array(x, y, z)
        - parent: The parent node. (Node)"""

    def __init__(self, position, parent):
        self.position = position
        self.parent = parent



class RRT_Dynamic:
    """A class to represent a Rapidly-Exploring Random Tree (RRT).
        - goal_pos: The position of the goal node. (x, y, z)
        - goal_threshold: The distance threshold for the goal. (float)
        - field_dimensions: The dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        - max_iterations: The maximum number of iterations to run the RRT algorithm. (int)
        - max_step_size: The maximum step size for the RRT algorithm. (float)
        - n_obstacles: The number of obstacles to create. (int)
        - frac_dynamic_obstacles: The fraction of obstacles that are dynamic. (float)
        - robot_width: The width of the robot. (float)
        - plot: Whether or not to plot the RRT tree before running the simulation. (bool)
        - obstacles: The obstacles to use. (list)
        - turn_radius: The turn radius of the robot. (float)
        - plot_step: Whether or not to plot the RRT graph after every step during the simulation. (bool)
        """    

    def __init__(self, goal_pos, goal_threshold, field_dimensions, max_iterations, max_step_size, n_obstacles, frac_dynamic_obstacles, robot_width, plot=True, obstacles=[], turn_radius=0, plot_step=False):
        self.goal_pos = goal_pos
        self.goal_threshold = goal_threshold
        self.field_dimensions = field_dimensions
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.n_obstacles = n_obstacles
        self.n_dynamic_obstacles = int(frac_dynamic_obstacles * n_obstacles)
        self.robot_width = robot_width
        self.plot = plot
        self.plot_step = plot_step
        self.obstacles = obstacles
        self.turn_radius = turn_radius

        self.reached = False
        self.nodes = []
        self.goal_path = []

        self.generate_start_and_goal()

        self.initial_goal_threshold = goal_threshold
        self.initial_max_step_size = max_step_size
        self.total_distance = np.linalg.norm(np.subtract(self.goal_pos, self.start_pos))


    
    def generate_start_and_goal(self):
        """Generate a start and goal path for the RRT algorithm."""

        # Generate a random start position.
        x = np.random.uniform(self.field_dimensions[0][0], self.field_dimensions[0][1])
        y = np.random.uniform(self.field_dimensions[1][0], self.field_dimensions[1][1])
        z = np.random.uniform(self.field_dimensions[2][0], self.field_dimensions[2][1])
        self.start_pos = np.array([x, y, z])

        # Generate a random goal position.
        if self.goal_pos is None:
            x = np.random.uniform(self.field_dimensions[0][0], self.field_dimensions[0][1])
            y = np.random.uniform(self.field_dimensions[1][0], self.field_dimensions[1][1])
            z = np.random.uniform(self.field_dimensions[2][0], self.field_dimensions[2][1])
            self.goal_pos = np.array([x, y, z])

        self.offset = 1.5 * self.turn_radius
        self.final_goal_pos = None

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
            self.final_goal_pos = self.goal_pos
            self.goal_pos = np.array([self.goal_pos[0], self.goal_pos[1] - self.offset - 1, self.goal_pos[2]])
        
        else:
            self.nodes.append(Node(self.start_pos, None))


    def update_start(self, start_pos):
        """Update the start position and reset the RRT.
            - start_pos: The new start position.
        """
        # Set new start position and change thresholds
        self.start_pos = start_pos
        self.control_step_size_and_threshold(self.start_pos, self.goal_pos, self.total_distance)

        # Reset the RRT.
        self.nodes = []
        self.nodes.append(Node(start_pos, None))
        self.reached = False
        self.goal_path = []



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
        if (start_dist < safety_radius + circle.radius) or (goal_dist < safety_radius + circle.radius):
            return False

        # Check if the circle is inside another circle.
        for other_circle in self.obstacles:
            dist = np.linalg.norm(np.subtract(circle.position, other_circle.position))
            if dist < circle.radius + other_circle.radius:
                return False

        return True

    
    def create_circles(self):
        """Create random circles"""

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

                # Randomly generate speed of circle if it is a dynamic object and set speed to 0 if it is a static object
                if(count < self.n_dynamic_obstacles):
                    speed_x = random.uniform(-0.2,0.2)
                    speed_y = random.uniform(-0.2,0.2)
                    speed_z = 0
                else:
                    speed_x = 0
                    speed_y = 0
                    speed_z = 0
                
                x_trajectory = f"{x} + {speed_x} * t"
                y_trajectory = f"{y} + {speed_y} * t"
                z_trajectory = f"{radius}"
                trajectory = [x_trajectory, y_trajectory, z_trajectory]
                speed = np.array([speed_x, speed_y, speed_z])

                # Create circle object
                circle = Circle(position=[x, y, z], radius=radius, robot_width=self.robot_width, trajectory=trajectory, speed=speed)

                # If circle is not valid, create a new circle
                if not self.valid_circle(circle):
                    continue

                # add 1 to the count
                count += 1
                
                # Add circle to list of obstacles
                self.obstacles.append(circle)
                break
    

    def update_obstacles(self, t):
        """Update the position of the obstacles"""
        if t > 0:
            for obstacle in self.obstacles:
                if np.array_equal(obstacle.speed, np.array([0,0,0])):
                    continue
                obstacle.position += obstacle.speed * t

    
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

        # Add the final node to the goal path
        node = self.nodes[-1]
        self.goal_path.append(node)

        # Loop through the parents of the node until the start node is found
        while node.parent is not None:
            # Add the parent node to the start of the list
            node = node.parent
            self.goal_path.insert(0, node)
        
        if self.final_goal_pos is not None:
            # Add the goal_node to the end of the path
            self.goal_path.append(Node(self.final_goal_pos, self.nodes[-1]))

        return self.goal_path

    
    def control_step_size_and_threshold(self, current_location, goal_pos, total_distance):
        dist = np.linalg.norm(np.subtract(current_location, goal_pos))

        field_width = self.field_dimensions[0][1] - self.field_dimensions[0][0]
        minimal_goal_threshold = field_width * 0.03
        minimal_max_step_size = field_width * 0.02
        
        thresh_range = self.initial_goal_threshold - minimal_goal_threshold
        step_range = self.initial_max_step_size - minimal_max_step_size
        
        scale_factor = dist/total_distance

        self.goal_threshold = scale_factor * thresh_range + minimal_goal_threshold
        self.max_step_size = scale_factor * step_range + minimal_max_step_size


    def create_rrt(self, time):
        """Create a RRT."""
        
        # update positions for dynamic obstacles
        if time > 0:
            for obstacle in self.obstacles:
                if np.array_equal(obstacle.speed, np.array([0,0,0])):
                    continue
                obstacle.position += obstacle.speed * time

        if len(self.obstacles) == 0:
            # Creating circles for the obstacles in the environment
            self.create_circles()

        min_dist = np.inf

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

            dist = np.linalg.norm(np.subtract(self.goal_pos, new_pos))
            if dist < min_dist:
                min_dist = dist

            # Check if the goal has been reached and if so return self.reached = True
            if self.goal_reached(new_node):
                self.path_to_goal()
                self.reached = True        
                return self.reached
        
        # If the goal has not been reached after the maximum number of iterations, return self.reached = False
        return self.reached
    

    def run_rrt(self, time):
        """Run RRT algorithm"""

         # Create the RRT until the goal is reached
        print("\nLooking for a path to the goal...")
        reached = self.create_rrt(time)
        while not reached:
            print("Goal not reached, trying again...")
            reached = self.create_rrt(time)
        print("Goal reached!")

        # Plot graph of nodes and path to goal
        PlotGraph(self)
            


if __name__ == "__main__":

    # Set parameters
    goal_pos                = None
    max_iterations          = 1000
    max_step_size           = 2
    goal_threshold          = 0.5
    n_obstacles             = 5
    frac_dynamic_obstacles  = 0.5
    field_dimensions        = np.array([(-3, 3), (-3, 3), (0, 0)])
    robot_width             = 0.4
    plot                    = True

    # Create the RRT
    rrt = RRT_Dynamic(goal_pos               = goal_pos, 
                      goal_threshold         = goal_threshold, 
                      field_dimensions       = field_dimensions, 
                      max_iterations         = max_iterations, 
                      max_step_size          = max_step_size, 
                      n_obstacles            = n_obstacles,
                      frac_dynamic_obstacles = frac_dynamic_obstacles,
                      robot_width            = robot_width,
                      plot                   = plot
                      )

    # Run the RRT
    rrt.run_rrt(time=0)

    