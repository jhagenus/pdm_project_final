import time
import numpy as np
import matplotlib.pyplot as plt
import csv
import config
from rrt_static import RRT_Static
from utils.plot_graph import PlotGraph
from rrt import RRT
from rrt_star import RRTStar


def calculate_distance_to_goal(rrt):


    goal_distance = 0

    path = rrt.goal_path
    for i in range(len(path) - 1):
        first_pos = path[i].position
        second_pos = path[i + 1].position
        distance = np.sqrt((first_pos[0] - second_pos[0]) ** 2 + (first_pos[1] - second_pos[1]) ** 2)
        goal_distance += distance
    
    return goal_distance

def intersection(self, source_pos, target_pos):
        """Check line-sphere (circle) intersection
            - source_pos: The position of the source point. np.array(x, y, z)
            - target_pos: The position of the target point. np.array(x, y, z)"""

        # Calculate the direction vector and normalize it by dividing by the distance
        dirn = np.subtract(target_pos, source_pos).astype(float)
        dist = float(np.linalg.norm(dirn))

        # If the distance between the source and target nodes is 0, there can be no intersection
        if dist == 0:
            return False

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

def check_intersection(obstacles, source_pos, target_pos):
        """Check if the line segment between the source and target nodes intersects with an obstacle.
            - source_pos: The position of the source node. np.array(x, y, z)
            - target_pos: The position of the target node. np.array(x, y, z)"""

        # Loop through all the obstacles to check for intersection
        for obstacle in obstacles:
            if intersection(obstacle, source_pos, target_pos):
                return True

        return False

def star(init_goal_path, obstacles):
    """Create a RRT* based on the path of the RRT."""
    
    # Add the goal node and the parking node to the new path
    new_goal_path = [init_goal_path[-1]]

    # Remove the start and goal to make sure that the path will always include the parking node and the node to start driving forward
    goal_path = init_goal_path

    while True:
        # Check if the goal path is (nearly) empty
        if(len(goal_path) <= 1):
            break
        
        # Define the node that needs to be reached
        destination_node = goal_path[-1]
        
        print("Checking intersection...")
        print("Number of nodes in goal path: " + str(len(goal_path)))

        # Loop through all nodes in the goal path, starting from the first node
        for i, source_node in enumerate(goal_path):

            # If the destination node is not reachable from the current node, continue to the next node
            if check_intersection(obstacles, source_node.position, destination_node.position):
                continue

            # If the destination is reachable, add it the current node to the new goal path and remove all nodes after the current node from the goal path
            else:
                new_goal_path.insert(0, source_node)
                goal_path = goal_path[:i+1]
                break
    
    # Add the start node to the new path
    # new_goal_path.insert(0, init_goal_path[0])
    
    return new_goal_path


if __name__ == '__main__':

    with open('data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["n", "runtime rrt*", "runtime proposed", "distance rrt*", "distance proposed"])


    # set initial parameters
    start_pos           = np.array([-7, -7, 0])
    goal_pos            = np.array([7, 7, 0])
    max_iterations      = 10000
    max_step_size       = 1
    goal_threshold      = 0.1
    n_obstacles         = 40
    field_dimensions    = np.array([(-8, 8), (-8, 8), (0, 0)])
    robot_width         = 0.4
    turn_radius         = 0
    plot                = False


    # Initializing the RRT classes
    rrt_init = RRT_Static(goal_pos           = goal_pos, 
                     goal_threshold     = goal_threshold, 
                     field_dimensions   = field_dimensions, 
                     max_iterations     = max_iterations, 
                     max_step_size      = max_step_size, 
                     n_obstacles        = n_obstacles, 
                     robot_width        = robot_width, 
                     turn_radius        = turn_radius,
                     plot               = plot,
                     start_pos          = start_pos
                     )

    obstacle_list = [(ob.position[0], ob.position[1], ob.radius) for ob in rrt_init.obstacles]


    # Run the algorithms multiple times to get an average runtime and distance
    n_runs = 10
    for i in range(n_runs):

        rrt = RRT(
            start=start_pos[0:2].tolist(),
            goal=goal_pos[0:2].tolist(),
            rand_area=[field_dimensions[0][0], field_dimensions[1][1]],
            obstacle_list=obstacle_list,
            expand_dis=2,
            robot_radius=0.2,
            max_iter=max_iterations)

        rrt_star = RRTStar(
            start=start_pos[0:2].tolist(),
            goal=goal_pos[0:2].tolist(),
            rand_area=[field_dimensions[0][0], field_dimensions[1][1]],
            obstacle_list=obstacle_list,
            expand_dis=2,
            robot_radius=0.2,
            max_iter=max_iterations)


        # x_start = (start_pos[0], start_pos[1])  # Starting node
        # x_goal = (goal_pos[0], goal_pos[1])  # Goal node
        # step_len = max_step_size
        # goal_sample_rate = 0.05
        # max_iter = max_iterations
        # search_radius = goal_threshold


        # rrt_star = RrtStar(x_start=x_start, x_goal=x_goal, step_len=step_len, goal_sample_rate=goal_sample_rate, search_radius=search_radius, iter_max=max_iter, field_dimensions=field_dimensions, obstacles=rrt.obstacles, robot_radius=robot_width/2)


        # Calculating the runtime

        print("Runing rrt*")

        start_time_star = time.time()
        star_path = rrt_star.planning(animation=False)
        end_time_star = time.time()

        print("Running the proposed method")

        start_time_proposed = time.time()
        proposed_path = rrt.planning(animation=False)
        proposed_path = star(proposed_path, rrt_init.obstacles)
        end_time_proposed = time.time()
        

        # Create a new rrt and fill with the info of RRTStar to be able to create the same plots.
        rrt_star_new = RRT_Static(goal_pos           = goal_pos, 
                        goal_threshold     = goal_threshold, 
                        field_dimensions   = field_dimensions, 
                        max_iterations     = max_iterations, 
                        max_step_size      = max_step_size, 
                        n_obstacles        = n_obstacles, 
                        robot_width        = robot_width, 
                        turn_radius        = turn_radius,
                        plot               = plot,
                        start_pos          = start_pos
                        )
        rrt_proposed_new = RRT_Static(goal_pos           = goal_pos,
                            goal_threshold     = goal_threshold,
                            field_dimensions   = field_dimensions,
                            max_iterations     = max_iterations,
                            max_step_size      = max_step_size,
                            n_obstacles        = n_obstacles,
                            robot_width        = robot_width,
                            turn_radius        = turn_radius,
                            plot               = plot,
                            start_pos          = start_pos
                            )
        
        rrt_star_new.reset(goal_pos=goal_pos, start_pos=start_pos, obstacles=rrt_init.obstacles)
        rrt_star_new.nodes = rrt_star.node_list
        rrt_star_new.goal_path = star_path

        rrt_proposed_new.reset(goal_pos=goal_pos, start_pos=start_pos, obstacles=rrt_init.obstacles)
        rrt_proposed_new.nodes = rrt.node_list
        rrt_proposed_new.goal_path = proposed_path


        plot = True

        if plot:
            rrt_star_new.plot = plot
            rrt_proposed_new.plot = plot

            # Plot the graphs
            PlotGraph(rrt_star_new, name="RRT* method")
            PlotGraph(rrt_proposed_new, name="RRT using proposed optimization method")



        # Calculating the distance to the goal

        run_time_proposed = end_time_proposed - start_time_proposed
        run_time_star = end_time_star - start_time_star

        print("Calculating the distance to the goal")

        distance_proposed = calculate_distance_to_goal(rrt_proposed_new)
        distance_star = calculate_distance_to_goal(rrt_star_new)

        print("\n")

        print("RUNTIME OF THE DIFFERENT ALGORITHMS:")
        print(" - Proposed method: ", run_time_proposed)
        print(" - RRT*: ", run_time_star)

        print()
        print("DISTANCE TO THE GOAL:")
        print(" - Proposed method: ", distance_proposed)
        print(" - RRT*: ", distance_star)
        print()

        print("Saving data to csv file")

        with open("data.csv", 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([i, run_time_star, run_time_proposed, distance_star, distance_proposed])

