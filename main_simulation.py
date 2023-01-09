import gym
from urdfenvs.robots.prius import Prius
import numpy as np
import math
import time
import threading
import copy
from rrt_static import RRT
from TangentCalculator import find_tangent_points
from command_generator import controller
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from urdfenvs.sensors.lidar import Lidar
from urdfenvs.sensors.full_sensor import FullSensor

class runPrius():
    def __init__(self,n_steps=1000, render=False, goal=True,actions=[],parameters=[],obstacles=0):
        pos_x = -8
        pos_y = -8
        angle = 0
        self.actions = actions
        self.ready = False
        self.update = False
        self.max_steer = 0.9
        self.count = -1
        self.parameters = parameters
        self.obstacles = obstacles

        pos0 = np.array([pos_x, pos_y, angle*(math.pi/180)])

        robots = [
            Prius(mode="vel"),
        ]
        
        self.env = gym.make(
            "urdf-env-v0",
            dt=0.01, robots=robots, render=True
            )

        ob = self.env.reset(pos=pos0)
        self.generate_obstacles()

        self.drive_forward = np.array([1.0, 0.])
        self.drive_backward = np.array([-1.0, 0.])
        self.stop = np.array([0.,0.])
        self.turn_right =  np.array([0.0,-1.0])
        self.turn_left =  np.array([0.0,1.0])

        # Add lidar sensor to prius robot Lidar(link_id, amount of rays, length of rays)
        self.lidar = Lidar(8, nb_rays=4,ray_length=18.0,raw_data=True)
        self.env.add_sensor(self.lidar, robot_ids=[0])
        
        self.Switch = False

        x = threading.Thread(target=self.switcher) # initialise a thread
        x.start() # start the thread

        self.DEFAULT()        
    
    def generate_obstacles(self):
        """Generates the obstacles in the environment."""

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
        action = copy.deepcopy(self.stop)
        while(True):
            ob, _, _, _ = self.env.step(action)
            if self.update:
                self.update = False
                self.count += 1

                if self.count  == len(self.actions):
                    print("GOAL REACHED")
                    break
                
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
        # due to realllly stupid python behaivor indexing and changing the value of action will change the acctual self.turn_right array. PLEASE GIVE ME C++ BACK :(
        action = copy.deepcopy(self.turn_left)
        #boolean flags used for finite state machine
        NEXT = True
        NEXT2 = True
        NEXT3 = False
        Special_Case = False

        angle = self.parameters[self.count]

        #we get the initial orientation of the robot
        ob, _, _, _ = self.env.step(action)
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 # orientation goes from -180 to 180. we normalise values here
        if InitialOrientation +angle >= 360:
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
            if np.allclose(currentOrientation , InitialOrientation + angle,atol=0.8) and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True

            if NEXT2 and Special_Case and np.allclose(currentOrientation +360,angle,atol=0.8):
                action = copy.deepcopy(self.turn_right)
                NEXT3 = True
                

            #if your wheels have reach a neutral position ~0 degrees relative to car go to DEFAULT function and await next order of controller.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT2 = False
                NEXT3 = False
                break
            

            # print("current orientation: ",currentOrientation)
            # print("desired orientation: ",InitialOrientation + angle)
            # print("")


    def TURN_RIGHT(self):
        action = copy.deepcopy(self.turn_right)
        #boolean flags used for finite state machine
        NEXT = True
        NEXT2 = False
        NEXT3 = False
        Special_Case = False

        angle = self.parameters[self.count]

        #we get the initial orientation of the robot
        ob, _, _, _ = self.env.step(action)
        InitialOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) + 180 # orientation goes from -180 to 180. we normalise values here
        if InitialOrientation -angle <= 90 and InitialOrientation -angle > 0 :
            Special_Case = True
        #env.step(action) needs to be in a while loop otherwise the environment will end
        while(True):

            #current Orientation measures how far the robot currently is, currentSteerAngle does the same but for steering angle
            currentOrientation = ob['robot_0']['joint_state']['position'][2]*(180/math.pi) +180 # again normalised
            currentSteerAngle = ob['robot_0']['joint_state']['steering']
            ob, _, _, _ = self.env.step(action) #update the environment with the action
            
            #when the wheels are fully turned stop turning the wheels and start moving the car (needs to be this way because otherwise we cannot connect this to reeds shepp)
            if  abs(currentSteerAngle[0]) >= self.max_steer and NEXT:
                action[1] = 0
                action[0] = 1.0
                NEXT = False
                NEXT2 = True

            #if you have reached your orientation move your wheels back to a neutral position
            if np.allclose(InitialOrientation - angle, currentOrientation,atol=0.8)  and NEXT2 and not Special_Case:
                action = copy.deepcopy(self.turn_left)
                NEXT2 = False
                NEXT3 = True
              
            if Special_Case and NEXT2 and np.allclose(currentOrientation , (360 - angle),atol=0.8) and currentOrientation <= 360:
                action = copy.deepcopy(self.turn_left)
                NEXT2 = False
                NEXT3 = True
                
            #if your wheels have reach a neutral position ~0 degrees relative to car go to DEFAULT function and await next order of controller.
            if np.allclose(currentSteerAngle,0,atol=0.01) and NEXT3:
                NEXT3 = False
                break
            
            # print("special case:        ",Special_Case)
            # print("current orientation: ",currentOrientation)
            # print("desired orientation: ",InitialOrientation - angle)
            # print("3 consecutive checks: ",np.allclose((currentOrientation - angle ), InitialOrientation,atol=0.8), NEXT2 ,not Special_Case)
            # print("")
            

    def Forward(self):
        # find the node location by indexing at self.count (which contains the index of the current action)
        #do this in the list which contain the node locations to obtain the node location assosiated with the current action
        #take the euclidan distance of this location and of your current location
        #keep looping untill car is within some tollerance of the next node

        action = copy.deepcopy(self.drive_forward)
        ob, _, _, _ = self.env.step(action)
        desired_distance = self.parameters[self.count]
        current_dist = 0

        start_loc = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]

        while(not np.allclose(current_dist,desired_distance, atol=0.01)):
            ob, _, _, _ = self.env.step(action)
            current_location = [ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1]]
            current_dist = math.sqrt((current_location[0]-start_loc[0])**2+(current_location[1]-start_loc[1])**2)

            # print("current_dist: ",current_dist)
            # print("desired_distance: ",desired_distance)
            # print("")

            test = self.lidar._distances
   




    def switcher(self):
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
    n_obstacles = 20
    field_dimensions = np.array([(-9, 9), (-9, 9), (0, 0)])
    robot_radius = 0.2
    turn_radius = 1.37
    plot = True

    #generate RRT
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

    # Run the RRT algorithm and terminate if the goal has not been reached
    reached = rrt.run_rrt_star()
    if not reached:
        print("No path found!")
    else:
        # Get the path from start to goal
        reed_shepp_nodes = [node.position for node in rrt.goal_path]

        """
        interpolate the found nodes so that it can work with reed-shepp paths
        """
        # reed_shepp_nodes = [rrt_nodes[0]] 
        # for i in range(len(rrt_nodes)-2):
        #     p1 = rrt_nodes[i+1]
        #     p2 = rrt_nodes[i]
        #     p3 = rrt_nodes[i+2]
        #     node1,node2,center=find_tangent_points(p1,p2,p3,turn_radius)
        #     reed_shepp_nodes.append(node1)
        #     reed_shepp_nodes.append(node2)
        # reed_shepp_nodes.append(rrt_nodes[-1])

    
        #set of actions and correcponding parameters (distance or angle) for car to follow
        actions, parameters = controller(reed_shepp_nodes,turn_radius)

        # # Create the environment
        # env = Environment(field_dimensions=field_dimensions, 
        #                   obstacles=rrt.obstacles, 
        #                   robots=robots, 
        #                   render=render, 
        #                   start_pos=start_pos)
        
        Run_simulation= runPrius(actions=actions,parameters=parameters,obstacles=rrt.obstacles) #initialise the car

        # #threading example: https://realpython.com/intro-to-python-threading/
    



    

