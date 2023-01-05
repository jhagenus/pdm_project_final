import math
import numpy as np
import matplotlib.pyplot as plt


def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
  

def dot(vA, vB):
    return vA[0]*vB[0]+vA[1]*vB[1]


def ang(p1, p2, p3):
    # Get nicer vector form
    vA = [(p1[0]-p2[0]), (p1[1]-p2[1])]
    vB = [(p3[0]-p2[0]), (p3[1]-p2[1])]
    # Get dot prod
    dot_prod = dot(vA, vB)
    # Get magnitudes
    magA = dot(vA, vA)**0.5
    magB = dot(vB, vB)**0.5
    # Get angle in radians
    angle = math.acos(dot_prod/magB/magA)

    return angle


def find_tangent_points(p1, p2, p3, radius):
  """Find the tangent points between a circle and 2 lines."""
  
  # Calculate the direction of the lines
  A_dirn = (p1-p2)/distance(p1,p2)
  B_dirn = (p3-p2)/distance(p3,p2)
  circle_center_dirn = (A_dirn + B_dirn)/2

  # Calculate the angle between the lines
  angle = ang(p1, p2, p3)

  alpha = angle / 2
  dist_circle_center = radius / np.sin(alpha)   # Distance from p2 to circle center
  dist_circle_intersection = dist_circle_center * np.cos(alpha)  # Distance from p2 to circle intersection

  # Calculate the tangent points on the circle and the circle center
  A = p2 + dist_circle_intersection * A_dirn
  B = p2 + dist_circle_intersection * B_dirn
  circle_center = p2 + dist_circle_center * circle_center_dirn

  A = np.append(A, 0)
  B = np.append(B, 0)

  return A, B, circle_center


# # These are our nodes or in this case manually appointed coordinates
# pa = np.array([10,4])
# pb = np.array([0,0])
# pc = np.array([5,-2])

# position_A, position_B, center = find_tangent_points(pa, pb, pc, 1.37)
# print(center)
# print(position_A)


# # Visualize some results
# line1x = [pa[0],pb[0]]
# line1y = [pa[1],pb[1]]

# line2x = [pb[0],pc[0]]
# line2y = [pb[1],pc[1]]

# plt.plot(line1x,line1y)
# plt.plot(line2x,line2y)

# plt.plot(position_A[0], position_A[1], marker="o", color="blue")
# plt.plot(position_B[0], position_B[1], marker="o", color="blue")

# circle = plt.Circle(center[:2], 1.37, color='g', clip_on=False)
# plt.gca().add_patch(circle)
# plt.show()
