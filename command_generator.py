import math
import numpy as np

"""
This function takes as input the nodes for which dubins path works and a turning radius of the car.
The output of this controller is a list with actions: "drive_forward", "turn_left", "turn_right"
and a list with parameters associated with the actions. For forward motion this parameter is 
distance, for turning this parameter is the turning angle.
"""

def controller(dubins_nodes,radius):
    # Find first entry of the list parameters
    node1_location = dubins_nodes[0]
    node2_location = dubins_nodes[1]

    # Get the eculidan distance between these two nodes
    distance = math.sqrt((node2_location[0]-node1_location[0])**2+(node2_location[1]-node1_location[1])**2)

    # Initialise the list with action with the forward action
    # and the list with parmaters with this distance.
    actions=["drive_forward"]
    parameters = [distance]

    turn = True 

    for i in range(len(dubins_nodes)-2):
        # Loop through all the nodes in the list with dubins nodes to figure out the action that needs to be taken at each node
        reference_node = dubins_nodes[i] 
        first_node = dubins_nodes[i+1] 
        second_node = dubins_nodes[i+2]

        x0 = reference_node[0]
        y0 = reference_node[1]
        x1 = first_node[0]
        y1 = first_node[1]
        x2 = second_node[0]
        y2 = second_node[1]

        position = np.sign((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
        distance = math.sqrt((x2-x1)**2+(y2-y1)**2)

        if turn:

            #We find the angle between the two points using the cosine formula:
            domain = (2*(radius**2)-distance**2)/(2*(radius**2))
            alpha = math.acos(domain)
            alpha = 180*alpha/(math.pi) # In degrees
           
            turn = False
 
            if (position == 1):
                actions.append("turn_left")
                parameters.append(alpha)
                
            else:
                actions.append("turn_right")
                parameters.append(alpha)

        # The car will go straight, turn and go straight again, this keeps repeating, meaning that if the car does
        # not need to turn we need to go straight.
        else:
            actions.append("drive_forward")
            parameters.append(distance)
            turn = True
            
    return actions, parameters
