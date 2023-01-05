import gym
from urdfenvs.robots.prius import Prius
import numpy as np
import math
import time
import threading
import copy
<<<<<<< Updated upstream
=======
from rrt_static import RRT
from TangentCalculator import find_tangent_points
from command_generator import controller
>>>>>>> Stashed changes


class runPrius():
    def __init__(self,n_steps=1000, render=False, goal=True, obstacles=True,actions=[],nodes=[]):
        pos_x = 0
        pos_y = 0
        angle = 90
        self.actions = actions
        self.ready = False
        self.update = False
        self.max_steer = 0.9
        self.count = 0
        self.nodes = nodes

        pos0 = np.array([pos_x, pos_y, angle*(math.pi/180)])

        robots = [
            Prius(mode="vel"),
        ]
        
        self.env = gym.make(
            "urdf-env-v0",
            dt=0.01, robots=robots, render=True
            )

        ob = self.env.reset(pos=pos0)

        self.drive_forward = np.array([1.0, 0.])
        self.drive_backward = np.array([-1.0, 0.])
        self.stop = np.array([0.,0.])
        self.turn_right =  np.array([0.0,-1.0])
        self.turn_left =  np.array([0.0,1.0])

        self.Switch = False

        x = threading.Thread(target=self.switcher) # initialise a thread
        x.start() # start the thread

        self.DEFAULT()

    def DEFAULT(self):
        action = copy.deepcopy(self.stop)
        while(True):
            ob, _, _, _ = self.env.step(action)
            if self.update:
                self.update = False
                if self.count  == len(self.actions):
                    print("GOAL REACHED")
                    break
                next_action = self.actions[self.count]
                print("next action is: ",next_action)
                self.count += 1
                if next_action == "turn_right":
                    self.TURN_RIGHT()
                elif next_action == "turn_left":
                    self.TURN_LEFT()
                elif next_action == "drive_forward":
                    self.Forward()
            else:
                self.ready = True
         
    def TURN_LEFT(self):
        # due to realllly stupid python behaivor indexing and changing the value of action will change the acctual self.turn_right array. PLEASE GIVE ME C++ BACK :(
        action = copy.deepcopy(self.turn_left)
        #boolean flags used for finite state machine
        NEXT = True
        NEXT2 = True
        NEXT3 = False
        Special_Case = False

        #we get the initial orientation of the robot
        ob, _, _, _ = self.env.step(action)
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 # orientation goes from -180 to 180. we normalise values here
        if InitialOrientation >= 269:
            Special_Case = True

        #env.step(action) needs to be in a while loop otherwise the environment will end
        while(True):
            
            #current Orientation measures how far the robot currently is, currentSteerAngle does the same but for steering angle
            currentOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) +180 # again normalised
            currentSteerAngle = ob['robot_0']['joint_state']['steering']
            ob, _, _, _ = self.env.step(action) #update the environment with the action

            #when the wheels are fully turned stop turning the wheels and start moving the car (needs to be this way because otherwise we cannot connect this to reeds shepp)
            if  currentSteerAngle > self.max_steer and NEXT:
                action[1] = 0
                action[0] = 1.0
                NEXT = False

            #if you have reached your orientation move your wheels back to a neutral position
            if currentOrientation >= InitialOrientation + 90 and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True

            if NEXT2 and Special_Case and currentOrientation >= 0 and currentOrientation <=90:
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True
                

            #if your wheels have reach a neutral position ~0 degrees relative to car go to DEFAULT function and await next order of controller.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT2 = False
                NEXT3 = False
                break


    def TURN_RIGHT(self):
        action = copy.deepcopy(self.turn_right)
        #boolean flags used for finite state machine
        NEXT = True
        NEXT2 = True
        NEXT3 = False
        Special_Case = False

        #we get the initial orientation of the robot
        ob, _, _, _ = self.env.step(action)
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 # orientation goes from -180 to 180. we normalise values here
        if InitialOrientation < 91 and InitialOrientation >= 0 :
            Special_Case = True
        #env.step(action) needs to be in a while loop otherwise the environment will end
        while(True):

            #current Orientation measures how far the robot currently is, currentSteerAngle does the same but for steering angle
            currentOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) +180 # again normalised
            currentSteerAngle = ob['robot_0']['joint_state']['steering']
            Continue = True
            ob, _, _, _ = self.env.step(action) #update the environment with the action

        
            
            #when the wheels are fully turned stop turning the wheels and start moving the car (needs to be this way because otherwise we cannot connect this to reeds shepp)
            if  abs(currentSteerAngle[0]) >= self.max_steer and NEXT:
                action[1] = 0
                action[0] = 1.0
                NEXT = False

            #if you have reached your orientation move your wheels back to a neutral position
            if currentOrientation +90 <= InitialOrientation  and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_left)
                NEXT3 = True
              
            if Special_Case and NEXT2 and currentOrientation >= 269 and currentOrientation <= 360:
                action = copy.deepcopy(self.turn_left)
                NEXT3 = True
                
            #if your wheels have reach a neutral position ~0 degrees relative to car go to DEFAULT function and await next order of controller.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT2 = False
                NEXT3 = False
                break

    def Forward(self):
        # find the node location by indexing at self.count (which contains the index of the current action)
        #do this in the list which contain the node locations to obtain the node location assosiated with the current action
        #take the euclidan distance of this location and of your current location
        #keep looping untill car is within some tollerance of the next node

        action = copy.deepcopy(self.drive_forward)
        ob, _, _, _ = self.env.step(action)
<<<<<<< Updated upstream
        node_location = self.nodes[self.count]
        node_distance = math.sqrt(node_location[0]**2+node_location[1]**2)
        current_location = math.sqrt(ob['robot_0']['joint_state']['position'][0]**2+ob['robot_0']['joint_state']['position'][1]**2)
        
        print("node_distance",node_distance)
        print("current_location",current_location)

        while(not np.allclose(node_distance,current_location, atol=0.01)):
            ob, _, _, _ = self.env.step(action)
            current_location = math.sqrt(ob['robot_0']['joint_state']['position'][0]**2+ob['robot_0']['joint_state']['position'][1]**2)
            print("node_distance",node_distance)
            print("current_location",current_location)
=======
        desired_distance = self.nodes[self.count]
        current_dist = 0

        start_loc = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]

        while(not np.allclose(current_dist,desired_distance, atol=0.01)):
            ob, _, _, _ = self.env.step(action)
            current_location = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]
            current_dist = math.sqrt((current_location[0]-start_loc[0])**2+(current_location[1]-start_loc[1])**2)
>>>>>>> Stashed changes



    def switcher(self):
        while(True):
            if self.ready:
                self.update = True
                self.ready = False
            else:
                time.sleep(0.5)
                #print("thread still running")

                
                

if __name__ == "__main__":
<<<<<<< Updated upstream
    #set of actions return by reeds sheep rrt implementation
    reeds_shepp = ["drive_forward","turn_right","turn_left"]
    node_locations =[np.array([2.0,2.0]),np.array([1.5,0.1]),np.array([0.1,1.1])]

    AUTO_car = runPrius(actions=reeds_shepp,nodes=node_locations) #initialise the car

    #threading example: https://realpython.com/intro-to-python-threading/
=======

    """
    first we create a rrt* like path
    """
    # Set parameters
    start_pos = np.array([-8, -8, 0])
    goal_pos = np.array([8, 8, 0])
    max_iterations = 1000
    max_step_size = 3
    goal_threshold = 0.2
    n_obstacles = 2
    field_dimensions = np.array([(-9.9, 9.9), (-9.9, 9.9), (0, 0)])
    robot_radius = 0.2
    plot = True
    render = True
    radius = 1.37


    #generate RRT
    rrt = RRT(start_pos=start_pos, 
            goal_pos=goal_pos, 
            goal_thresh=goal_threshold, 
            field_dimensions=field_dimensions, 
            max_iterations=max_iterations, 
            max_step_size=max_step_size, 
            n_obstacles=n_obstacles, 
            robot_radius=robot_radius, 
            plot=plot)

    # Run the RRT algorithm and terminate if the goal has not been reached
    reached = rrt.run_rrt_star()
    if not reached:
        print("No path found!")
    
    # Get the path from start to goal
    rrt_nodes = [node.position for node in rrt.goal_path]


    """
    interpolate the found nodes so that it can work with reed-shepp paths
    """

    reed_shepp_nodes = [rrt_nodes[0]] 
    for i in range(len(rrt_nodes)-2):
        p1 = rrt_nodes[i+1]
        p2 = rrt_nodes[i]
        p3 = rrt_nodes[i+2]
        node1,node2,center=find_tangent_points(p1,p2,p3,radius)
        reed_shepp_nodes.append(node1)
        reed_shepp_nodes.append(node2)
    reed_shepp_nodes.append(rrt_nodes[-1])

    print(reed_shepp_nodes)

    actions, parameters = controller(reed_shepp_nodes,radius)

    print(actions)
    print(parameters)

    # #set of actions return by reeds sheep rrt implementation
    # reeds_shepp = ["turn_right","turn_left"]
    # node_locations =[np.array([2.0,2.0]),np.array([1.5,0.1])]

    # Run_simulation= runPrius(actions=reeds_shepp,nodes=node_locations) #initialise the car

    # #threading example: https://realpython.com/intro-to-python-threading/
>>>>>>> Stashed changes
   



   

