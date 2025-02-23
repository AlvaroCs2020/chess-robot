import math

class IkUtils:
    def __init__(self, length1: float, length2: float):
        """
        Initializes the inverse kinematics utility with the two link lengths.
        :param length1: Length of the first link
        :param length2: Length of the second link
        """
        self.l1 = length1
        self.l2 = length2

    def compute_angles(self, x: float, y: float):
        """
        Computes the inverse kinematics for a 2DOF arm.
        :param x: Target x-coordinate
        :param y: Target y-coordinate
        :return: (theta1, theta2) in degrees
        """
        distance = math.sqrt(x**2 + y**2)
        
        if distance > self.l1 + self.l2:
            raise ValueError("Target is out of reach")
        
        cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        theta2 = math.acos(cos_theta2)
        
        k1 = self.l1 + self.l2 * math.cos(theta2)
        k2 = self.l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return math.degrees(theta1), math.degrees(theta2)
