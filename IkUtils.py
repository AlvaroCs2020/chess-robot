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

    #def compute_angles(self, x: float, y: float):
    #    """
    #    Computes the inverse kinematics for a 2DOF arm.
    #    :param x: Target x-coordinate
    #    :param y: Target y-coordinate
    #    :return: (theta1, theta2) in degrees
    #    """
    #    distance = math.sqrt(x**2 + y**2)
    #    
    #    if distance > self.l1 + self.l2:
    #        raise ValueError("Target is out of reach")
    #    
    #    cos_theta2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
    #    theta2 = math.acos(cos_theta2)
    #    
    #    k1 = self.l1 + self.l2 * math.cos(theta2)
    #    k2 = self.l2 * math.sin(theta2)
    #    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    #    
    #    return math.degrees(theta1) , math.degrees(theta2) +90
    def compute_angles(self, x: float, y: float):
        """
        Calcula la cinemática inversa para un brazo de 2DOF usando los mismos pasos que la función de Arduino.
        Se asume que Z es 0.
        :param x: Coordenada x objetivo.
        :param y: Coordenada y objetivo.
        :return: (ángulo hombro, ángulo codo) en grados.
        """

        # Rotar el punto al plano XY
        newX = x
        newY = y

        # Calcular la distancia en el plano resultante
        length2 = math.sqrt(newX * newX + newY * newY)

        # Calcular el ángulo para el hombro (angle0) usando el teorema del coseno
        cosAngle0 = ((length2 ** 2) + (self.l1 ** 2) - (self.l2 ** 2)) / (2 * length2 * self.l1)
        # Por precaución, limitar el valor entre -1 y 1 antes de aplicar acos
        cosAngle0 = max(min(cosAngle0, 1), -1)
        angle0 = math.degrees(math.acos(cosAngle0))

        # Calcular el ángulo para el codo (angle1) usando el teorema del coseno
        cosAngle1 = ((self.l2 ** 2) + (self.l1 ** 2) - (length2 ** 2)) / (2 * self.l2 * self.l1)
        cosAngle1 = max(min(cosAngle1, 1), -1)
        angle1 = math.degrees(math.acos(cosAngle1))

        # Calcular el ángulo de la posición en el plano
        atang = math.degrees(math.atan(newY / newX)) if newX != 0 else (90.0 if newY >= 0 else -90.0)
        jointAngle0 = atang + angle0  # Ángulo del hombro
        jointAngle1 = 180.0 - angle1  # Ángulo del codo

        # Si newX es negativo, ajustar el ángulo del hombro
        if newX < 0:
            jointAngle0 = abs(180 + jointAngle0)

        # Verificación de alcance: si la posición está fuera del alcance, se asignan valores de error
        if self.l1 + self.l2 < length2:
            jointAngle0 = atang
            jointAngle1 = 0

        # Se retorna el ángulo del hombro y el ángulo del codo (se le suma 90 al ángulo del codo, como en la función Arduino)
        return jointAngle1+90, jointAngle0
#ikTools = IkUtils(200,150)
#angle1, angle2 = ikTools.compute_angles(0,350)
#print(str(angle1) + " : " + str(angle2))
#angle1, angle2 = ikTools.compute_angles(150,200)
#print(str(angle1) + " : " + str(angle2))
#
#angle1, angle2 = ikTools.compute_angles(350,0)
#print(str(angle1) + " : " + str(angle2))
#angle1, angle2 = ikTools.compute_angles(250,0)
#print(str(angle1) + " : " + str(angle2))

