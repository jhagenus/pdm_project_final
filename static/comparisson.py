import time
import numpy as np
import matplotlib.pyplot as plt
import config
from rrt_star import RRTStar
from rrt_static import RRT_Static
from utils.plot_graph import PlotGraph


if __name__ == '__main__':

    # set initial parameters
    start_pos           = np.array([-7, -7, 0])
    goal_pos            = np.array([7, 7, 0])
    max_iterations      = 500
    max_step_size       = 2
    goal_threshold      = 2
    n_obstacles         = 10
    field_dimensions    = np.array([(-8, 8), (-8, 8), (0, 0)])
    robot_width         = 0.4
    turn_radius         = 0
    plot                = False


    # Create the RRT
    rrt = RRT_Static(goal_pos           = goal_pos, 
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

    obstacle_list = [(ob.position[0], ob.position[1], ob.radius) for ob in rrt.obstacles]

    rrt_star = RRTStar(
        start=start_pos[0:2].tolist(),
        goal=goal_pos[0:2].tolist(),
        rand_area=[field_dimensions[0][0], field_dimensions[1][1]],
        obstacle_list=obstacle_list,
        expand_dis=2,
        robot_radius=0.2,
        max_iter=500,)




    start_time_star = time.time()

    path = rrt_star.planning(animation=False)

    end_time_star = time.time()


    start_time_proposed = time.time()

    rrt.run_rrt(dubins=False)

    end_time_proposed = time.time()

    print("\n\n\n")

    print("RUNTIME OF THE DIFFERENT ALGORITHMS:\n")
    print("- Time taken for proposed method: ", end_time_proposed - start_time_proposed)
    print("- Time taken for RRT*: ", end_time_star - start_time_star)

    print("\n")

    # Plot the graphs
    plot = True
    rrt.plot = plot

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
    rrt_star_new.reset(goal_pos=goal_pos, start_pos=start_pos, obstacles=rrt.obstacles)
    rrt_star_new.nodes = rrt_star.node_list
    rrt_star_new.goal_path = path


    # Plot the graphs
    PlotGraph(rrt, name="Proposed optimization method")
    PlotGraph(rrt_star_new, name="RRT* method")

