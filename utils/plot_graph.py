from matplotlib import pyplot as plt
from utils.obstacles import Circle


class PlotGraph:
    """Plot graph of nodes and path to goal.
        - nodes: List of nodes in the RRT.
        - field_dimensions: Dimensions of the field. [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
        - start_pos: The start position of the robot. (x, y, z)
        - goal_pos: The goal position of the robot. (x, y, z)
        - obstacles: List of obstacles in the environment.
        - goal_path: List of nodes from start to goal by following parent nodes.
        - plot: Boolean that determines if the graph should be plotted or not. (Bool: True)"""
        
    def __init__(self, nodes, field_dimensions, start_pos, goal_pos, obstacles, goal_path, plot=True):
        self.nodes = nodes
        self.field_dimensions = field_dimensions
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.obstacles = obstacles
        self.goal_path = goal_path
        self.plot = plot

        if self.plot:
            self.create_graph()
        
        
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
        plt.plot(self.start_pos[0], self.start_pos[1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red")
        plt.plot(self.goal_pos[0], self.goal_pos[1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red")
        
        # Plot nodes as green dots
        for node in self.nodes[1:]:
            x = node.position[0]
            y = node.position[1]
            plt.plot(x, y, marker="o", markersize=3, markerfacecolor="blue", markeredgecolor="blue")
    

    def plot_tree(self):
        """Plot all edges between nodes"""

        for node in self.nodes:
            if node.parent is not None:
                source = node.parent.position
                target = node.position
                plt.plot([source[0], target[0]], [source[1], target[1]], 'b-')


    def plot_path(self):
        """Plot path to goal"""

        for i in range(len(self.goal_path)-1):

            # Plot line between nodes in the path to goal
            source = self.goal_path[i].position
            target = self.goal_path[i+1].position
            plt.plot(source[0], source[1], marker="o", markersize=5, markerfacecolor="red", markeredgecolor="red")
            plt.plot(target[0], target[1], marker="o", markersize=5, markerfacecolor="red", markeredgecolor="red")
            plt.plot([source[0], target[0]], [source[1], target[1]], 'r-', linewidth=3)

    
    def create_graph(self):
        """Create graph of nodes and path to goal"""

        # set plot parameters
        plt.rcParams["figure.figsize"] = [10, 10]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(self.field_dimensions[0][0]-1, self.field_dimensions[0][1]+1)
        plt.ylim(self.field_dimensions[1][0]-1, self.field_dimensions[1][1]+1)
        plt.grid()

        # plot nodes, path to goal and obstacles
        self.plot_obstacles()
        self.plot_nodes()
        self.plot_tree()
        self.plot_path()
        plt.show()