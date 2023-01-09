import math
import numpy as np
import matplotlib.pyplot as plt


def distance(p1, p2):
    """ Compute Euclidean distance between two points"""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
  

def dot_product(vA, vB):
    """ Compute the dot product between two vectors"""
    return np.dot(vA, vB) #vA[0]*vB[0]+vA[1]*vB[1]


def angle_between_lines(p1, p2, p3):
    """ Compute the angle between two lines originating """
    """ in point 1, point 3 intersecting in point 2     """
    # Obtain vectorial form of the two lines
    vA = [(p1[0]-p2[0]), (p1[1]-p2[1])]
    vB = [(p3[0]-p2[0]), (p3[1]-p2[1])]
    
    # Compute the dot product between the two vectors
    dot_prod = dot_product(vA, vB)
    
    # Compute the magnitude of the two vectors respectively
    magnitude_A = dot_product(vA, vA)**0.5
    magnitude_B = dot_product(vB, vB)**0.5
    
    # Compute the angle between two vectors in radians
    angle = math.acos(dot_prod/magnitude_B/magnitude_A)

    return angle


def find_tangent_points(p1, p2, p3, radius):
  """ Implementation of the Tangent Tanget intersection theorem"""
  
  # Determine the direction of the two intersecting lines and the midpoint line running through the circle center
  direction_A = (p1-p2)/distance(p1,p2)
  direction_B = (p3-p2)/distance(p3,p2)
  circle_center_dirn = (direction_A + direction_B)/2

  # Compute the angle between the two lines
  angle = angle_between_lines(p1, p2, p3)

  # Divide the angle by two to obtain angle between line through circle center and either of the two tangent lines
  alpha = angle / 2
  distance_to_circle_center = radius / np.sin(alpha)   # Distance from p2 to circle center
  distance_to_circle_intersection = distance_to_circle_center * np.cos(alpha)  # Distance from p2 to circle intersection

  # Calculate the tangent points on the circle and the circle center
  A = p2 + distance_to_circle_intersection * direction_A
  B = p2 + distance_to_circle_intersection * direction_B
  circle_center = p2 + distance_to_circle_center * circle_center_dirn

  A = np.append(A, 0)
  B = np.append(B, 0)

  return A, B, circle_center