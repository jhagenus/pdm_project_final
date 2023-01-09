# Import package dependencies

from MotionPlanningEnv.sphereObstacle import SphereObstacle
from command_generator import controller
from urdfenvs.robots.prius import Prius
from rrt_static import RRT
import numpy as np
import threading
import time
import copy
import math
import gym


class runPrius():

    # Class constructor
    def __init__(self, rrt, actions=[],parameters=[]):
        starting_position_x, starting_position_y, starting_position_z  = rrt.start_pos
        starting_orientation = rrt.start_orientation
        self.actions         = actions
        self.ready           = False
        self.update          = False
        self.max_steer       = 0.9
        self.count           = -1
        self.parameters      = parameters
        self.obstacles       = rrt.obstacles

        # Implement the Prius Robot using the Bicycle model and create our gym environment
        robots = [Prius(mode="vel")]
        
        #initialise the environment
        self.env = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=True)
        starting_position = np.array([starting_position_x, starting_position_y, starting_orientation])
        ob = self.env.reset(pos=starting_position)
        self.generate_obstacles()

        # Define discrete action inputs for our car controller: action = [forward_velocity, angular_velocity]
        # the actions are discretized in order to respect the kinematic constraints implemented in the bicycle model
        self.drive_forward  = np.array([1.0, 0.])
        self.stop           = np.array([0.0,0.0])
        self.turn_right     = np.array([0.0,-1.0])
        self.turn_left      = np.array([0.0,1.0])
        
        # Boolean flag to switch between threads, python uses single thread by default. In this code multithreading was
        # implemented to switch between processes. 
        self.Switch = False
        x = threading.Thread(target=self.switcher) # initialise a thread
        x.start() # start the thread


        #A finite state machine is implemented using this class with the first state being default.
        self.DEFAULT()        
    
    
    def generate_obstacles(self):
        """Function which generates the obstacle for the gym """
        """environment using randomly initialized positions. """
           
        for obstacle in self.obstacles:
            radius = float(obstacle.radius)
            position = [obstacle.position[0],obstacle.position[1], radius]
            position = np.array(position, dtype=float).tolist()

            # Create the obstacle dictionary with the position and radius and add it to the environment
            obs_dict = {
                "type": "sphere",
                'movable': True,
                "geometry": {"position": position, "radius": radius},
                }
            sphereObst = SphereObstacle(name="simpleSphere", content_dict=obs_dict)
            self.env.add_obstacle(sphereObst)

    def DEFAULT(self):
        """Default car action used to stop the vehicle in position"""
        action = copy.deepcopy(self.stop)
        while(True):
            ob, _, _, _ = self.env.step(action)

            # Here we check if the car has finished its command using the self.update boolean. 
            # This boolean is set using the switcher function where we use multitreadhing as a way
            # to switch between states in the finite state machine (it acts like a serivice interrupt routine)
            if self.update:
                self.update = False
                self.count += 1
                
                #if the last action in the list has been reached we have finished the track and finished the course
                if self.count  == len(self.actions):
                    print("GOAL REACHED")
                    break
                
                #we look through the list with actions to determine if we should turn in a direction or move straight
                next_action = self.actions[self.count]
                print("next action is: ",next_action)
                if next_action == "turn_right":
                    self.TURN_RIGHT()
                elif next_action == "turn_left":
                    self.TURN_LEFT()
                elif next_action == "drive_forward":
                    self.Forward()
            else:
                self.ready = True
            
         
    def TURN_LEFT(self):
        """Discretized action to turn the car to the left"""
        """with a steering angle ranging from 0 to 180 degrees"""
        action = copy.deepcopy(self.turn_left)
        # Boolean flags used for a finite state machine
        NEXT = True
        NEXT2 = True
        NEXT3 = False
        Special_Case = False

        # Look up the angle the car needs to turn to reach the next node 
        angle = self.parameters[self.count]

        # Get the initial state of the robot
        ob, _, _, _ = self.env.step(action)

        # The orientation goes from -180 to 180, these values are normalised to 0 to 360 degrees
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 

        # In the case that the steering of the robot will turn it past the 360/0 degree point a 
        # special care needs to be given to define how the robot needs to turn
        if InitialOrientation +angle >= 360:
            Special_Case = True

        while(True):
            
            # Current Orientation measures how far the robot currently is, currentSteerAngle does the same but for steering angle
            currentOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) +180 
            currentSteerAngle = ob['robot_0']['joint_state']['steering']
            ob, _, _, _ = self.env.step(action) 

            # Action gets updated to stop turning the wheels when the wheels have reached the maximum steering angle
            # and to start moving the car
            if  currentSteerAngle > self.max_steer and NEXT:
                action[1] = 0
                action[0] = 1.0
                NEXT = False

            # As soon as the car has reached its correct orrientation the wheels can turn back to 0 degrees relative to the car
            if np.allclose(currentOrientation , InitialOrientation + angle,atol=0.8) and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True

            # Special case mentioned earlier
            if NEXT2 and Special_Case and np.allclose(currentOrientation +360,angle,atol=0.8):
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True
                

            # If your wheels have reach a neutral position ~0 degrees relative to car go to DEFAULT function and await next order of controller.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT2 = False
                NEXT3 = False
                break



    def TURN_RIGHT(self):
        """Discretized action to turn the car to the right"""
        """with a steering angle ranging from 0 to 180 degrees"""
        
        action = copy.deepcopy(self.turn_right)
        #boolean flags used for finite state machine
        NEXT = True
        NEXT2 = False
        NEXT3 = False
        Special_Case = False

        angle = self.parameters[self.count]

        # The orientation goes from -180 to 180, these values are normalised to 0 to 360 degrees
        ob, _, _, _ = self.env.step(action)
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 # orientation goes from -180 to 180. we normalise values here
        if InitialOrientation -angle <= 90 and InitialOrientation -angle > 0 :
            Special_Case = True

        while(True):

            # Current Orientation measures how far the robot currently is, currentSteerAngle does the same but for steering angle
            currentOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) +180 # again normalised
            currentSteerAngle = ob['robot_0']['joint_state']['steering']
            ob, _, _, _ = self.env.step(action) #update the environment with the action
            
            # As soon as the wheels are fully turned in: stop turning the wheels and start moving the car 
            # (needs to be this way because otherwise we cannot connect this to Dubins Path Planning)
            if  abs(currentSteerAngle[0]) >= self.max_steer and NEXT:
                action[1] = 0
                action[0] = 1.0
                NEXT = False
                NEXT2 = True

            # In case we have reached the desired orientation: move your wheels back to the neutral position (e.g. 0 degree angle)
            if np.allclose(InitialOrientation - angle, currentOrientation,atol=0.8)  and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_left)
                NEXT2 = False
                NEXT3 = True
              
            if Special_Case and NEXT2 and np.allclose(currentOrientation , (360 - angle),atol=0.8) and currentOrientation <= 360:
                action = copy.deepcopy(self.turn_left)
                NEXT2 = False
                NEXT3 = True
                
            # In case your wheels have reached the neutral position e.g. 0 degrees(relative to car) enter Default state and await control command.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT3 = False
                break
            
            

    def Forward(self):
        """ Get node location by indexing at self.count (contains index of the current action)."""
        """ Apply to list with node locations to obtain current action associated with node location"""
        """ Determine euclidean distance between this location and your current location. """
        """ Repeat until car is within certain tolerance threshold of next node. """

        action = copy.deepcopy(self.drive_forward)
        ob, _, _, _ = self.env.step(action)
        desired_distance = self.parameters[self.count]
        current_dist = 0

        start_loc = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]

        while(not np.allclose(current_dist,desired_distance, atol=0.01)):
            ob, _, _, _ = self.env.step(action)
            current_location = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]
            current_dist = math.sqrt((current_location[0]-start_loc[0])**2+(current_location[1]-start_loc[1])**2)

   
    def switcher(self):
        """ Switcher function which assists the controller in switching"""
        """ between states if new control command is ready"""
        while(True):
            if self.ready:
                self.update = True
                self.ready = False
            else:
                time.sleep(0.05)
                #print("thread still running")

                

if __name__ == "__main__":

    """
    first we create a rrt* like path
    """
     # Set parameters
    start_pos = np.array([-8, -8, 0])
    goal_pos = np.array([8, 8, 0])
    max_iterations = 1000
    max_step_size = 2
    goal_threshold = 0.5
    n_obstacles = 10
    field_dimensions = np.array([(-9, 9), (-9, 9), (0, 0)])
    robot_radius = 0.5
    turn_radius = 1.37
    plot = True

    #generate RRT
    rrt = RRT(start_pos         = start_pos, 
              goal_pos          = goal_pos, 
              goal_thresh       = goal_threshold, 
              field_dimensions  = field_dimensions, 
              max_iterations    = max_iterations, 
              max_step_size     = max_step_size, 
              n_obstacles       = n_obstacles, 
              robot_radius      = robot_radius, 
              turn_radius       = turn_radius,
              plot              = plot)

    # Run the RRT algorithm and terminate if the goal has not been reached
    reached = rrt.run_rrt_star()
    if not reached:
        print("No path found!")
    else:
        # Get the path from start to goal
        dubins_nodes = [node.position for node in rrt.goal_path]

        # Generate a set of actions and correcponding parameters (distance or angle) for the car to follow
        actions, parameters = controller(dubins_nodes,turn_radius)
        
        # Initialising the class will run the simulation with the prius model.
        Run_simulation= runPrius(rrt=rrt, actions=actions,parameters=parameters) 

    
    



    

