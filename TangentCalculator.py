import numpy as np
import matplotlib.pyplot as plt
import math


# These are our nodes or in this case manually appointed coordinates
pa = np.array([8,-1])
pb = np.array([0,0])
pc = np.array([5,-2])

# Constructing a function to compute the angle between two intersecting lines, point P2 is the point of intersection
def find_tangent_points(p1, p2, p3, radius):
  # Calculate the slopes of the lines
  m1 = (p2[1] - p1[1]) / (p2[0] - p1[0])
  m2 = (p3[1] - p2[1]) / (p3[0] - p2[0])
  
  # Calculate the angle between the lines
  angle = abs(math.atan2(m2 - m1, 1 + m1 * m2))
  
  # The angles are being outputted in radians so we can use them directly in the next trigonometric functions

  # Make sure the angle is positive
  # if angle < 0:
  #   angle += 180
  
  # Construct our crossproduct equivalents in 2D
  ua = (pa-pb)/np.linalg.norm(pa-pb)
  ub = (pc-pb)/np.linalg.norm(pc-pb)

  # Compute the coordinates of point A and B respectively
  A = pb + radius*ua*(np.cos(angle/2)/np.sin(angle/2))
  B = pb + radius*ub*(np.cos(angle/2)/np.sin(angle/2))
  center = radius/np.sin(angle/2)

  return A, B, center

position_A, position_B, center = find_tangent_points(pa, pb, pc, 1.37)
print(center)




# Visualize some results
line1x = [pa[0],pb[0]]
line1y = [pa[1],pb[1]]

line2x = [pb[0],pc[0]]
line2y = [pb[1],pc[1]]

plt.plot(line1x,line1y)
plt.plot(line2x,line2y)

plt.plot(position_A[0], position_A[1], marker="o", color="blue")
plt.plot(position_B[0], position_B[1], marker="o", color="blue")

circle = plt.Circle([center-0.2, pa[1]-1.7], 1.37, color='g', clip_on=False)
plt.gca().add_patch(circle)
plt.show()
