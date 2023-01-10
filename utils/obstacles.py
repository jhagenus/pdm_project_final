import numpy as np


class Circle:
    """Circle class for obstacles.
        - position: The position of the center of the circle. (x, y, z)
        - radius: The radius of the circle. (float)
        - robot_width: The width of the robot. (float)
        - speed: The speed of the circle. (float)
        - trajectory: The trajectory of the circle. (np.array(x, y, z))
        - movable: Boolean that determines if the circle is movable or not."""

    def __init__(self, position, radius, robot_width, speed=0, trajectory=None, movable=True):
        self.position = position
        self.radius = radius
        self.robot_radius = robot_width / 2
        self.speed = speed
        self.trajectory = trajectory
        self.movable = movable
    

    def point_collision(self, point_pos):
        """Check if a point is inside the circle.
            - point_pos: The position of the point. (x, y, z)"""

        # Calculate the distance between the point and the circle.
        dist = np.linalg.norm(np.subtract(point_pos, self.position))

        # If the distance between the point and the circle is less than the radius, then the point is inside the circle.
        if dist < self.radius + self.robot_radius:
            return True

        return False


    def intersection(self, source_pos, target_pos):
        """Check line-sphere (circle) intersection
            - source_pos: The position of the source point. np.array(x, y, z)
            - target_pos: The position of the target point. np.array(x, y, z)"""

        # Calculate the direction vector and normalize it by dividing by the distance
        dirn = np.subtract(target_pos, source_pos)
        dist = np.linalg.norm(dirn)
        dirn /= dist # normalizDynamicSphereObstaclee

        # Add the robot radius to the circle radius to account for the robot's size
        radius = self.radius + self.robot_radius

        # Calculate the discriminant of the quadratic equation
        a = np.dot(dirn, dirn)
        b = 2 * np.dot(dirn, source_pos - self.position)
        c = np.dot(source_pos - self.position, source_pos - self.position) - radius * radius
        discriminant = b * b - 4 * a * c

        # If the discriminant is negative, there is no intersection
        if discriminant < 0:
            return False

        # Calculate the two intersection points
        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        # If both intersection points are outside the line segment, there is no intersection
        if (t1 < 0 and t2 < 0) or (t1 > dist and t2 > dist):
            return False

        return True