import numpy as np


def controller(dubins_path, radius):
    """The controller that is used to control the car. It generates the actions that the car needs to take to get from
       one node to the next together with the parameters associated with these actions. Distance for forward motion and
       turning angle for turning.
        - dubins_path: The nodes that are used to generate the dubins path. [(x, y), (x, y), ...]
        - radius: The turning radius of the car."""

    # Find first entry of the list parameters
    node1_location = dubins_path[0]
    node2_location = dubins_path[1]

    # Get the eculidan distance between these two nodes
    distance = np.linalg.norm(np.subtract(node2_location, node1_location))

    # Initialise actions with forward motion and the distance as the first parameter
    actions = ["drive_forward"]
    parameters = [distance]

    turn = True 

    # Loop through all the nodes in the Dubins path to figure out the action that needs to be taken at each node
    for i in range(len(dubins_path)-2):

        # Define the nodes that are used to generate the action
        reference_node = dubins_path[i] 
        first_node = dubins_path[i+1] 
        second_node = dubins_path[i+2]

        x0, y0 = reference_node[:2]
        x1, y1 = first_node[:2]
        x2, y2 = second_node[:2]

        # Determine the position of the second node relative to the line between the first and reference node and the distance between the two nodes
        position = np.sign((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        if turn:
            # Determine the angle between the two points using the cosine formula:
            domain = (2*(radius**2) - distance**2) / (2 * (radius**2))
            alpha = np.arccos(domain)
            alpha = 180*alpha/(np.pi) # In degrees
           
            turn = False

            # If the second node is on the left of the line between the first and reference node, the car needs to turn left
            if (position == 1):
                actions.append("turn_left")
                parameters.append(alpha)
            
            # If the second node is on the right of the line between the first and reference node, the car needs to turn right
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
