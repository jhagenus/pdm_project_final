import math
import numpy as np

def controller(reed_shepp_nodes,radius):
    actions=["drive_forward"]

    #find first entry of the list parameters
    node1_location = reed_shepp_nodes[0]
    node2_location = reed_shepp_nodes[1]

    distance = math.sqrt((node2_location[0]-node1_location[0])**2+(node2_location[1]-node1_location[1])**2)

    parameters = [distance]

    turn = True # car always turns, goes straights and turns. exploited by this boolean.

    for i in range(len(reed_shepp_nodes)-2):
        first_node = reed_shepp_nodes[i+1] # start node has no influence on action so it is filtered out.
        second_node = reed_shepp_nodes[i+2]

        x1 = first_node[0]
        y1 = first_node[1]
        x2 = second_node[0]
        y2 = second_node[1]

        distance = math.sqrt((x2-x1)**2+(y2-y1)**2)

        if turn:
            #We find the angle between the two points using the cosine formula:
            domain = (2*(radius**2)-distance**2)/(2*(radius**2))

            alpha = math.acos((2*(radius**2)-distance**2)/(2*(radius**2)))
            alpha = 180*alpha/(math.pi) # in degrees
           
            turn = False
            #conditional statement defines all left and right turns, this is a left turn. (intuition: draw xy graph and all possible configurations with first node at 0.0 and 90 degree max turns) 
            if ((x2 > x1) and (y2 > y1)) or ((x2> y1) and (y2 < x1)):
                actions.append("turn_left")
                parameters.append(alpha)
                
            else:
                actions.append("turn_right")
                parameters.append(alpha)

        #if the car does not turn we need the car to drive the distance between the two nodes
        else:
            actions.append("drive_forward")
            parameters.append(distance)
            turn = True
            
    return actions, parameters
