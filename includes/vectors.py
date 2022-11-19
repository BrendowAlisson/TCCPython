import numpy as np
import math

class Vector:
    def __init__(self):
        self.positionFingersTIP = {"pinky": 20, "ring": 16, "middle": 12, "index": 8, "thumb": 4}
        self.positionFingersMCP = {"pinky": 17, "ring": 13, "middle": 9, "index": 5, "thumb": 2}
        self.wristPosition = 0
    
    def create_finger_coordinates_vector(self, fingersCoordinates):
        self.vectorCoordinates = []
        for finger in self.positionFingersTIP:
            x = (fingersCoordinates[self.positionFingersTIP[finger]][1] - fingersCoordinates[self.positionFingersMCP[finger]][1])
            y = (fingersCoordinates[self.positionFingersTIP[finger]][2] - fingersCoordinates[self.positionFingersMCP[finger]][2])
            z = (fingersCoordinates[self.positionFingersTIP[finger]][3] - fingersCoordinates[self.positionFingersMCP[finger]][3])
            self.vectorCoordinates.append(np.array([x, y, z]))
        return self.vectorCoordinates

    def create_normal_vector(self, fingersCoordinates):
        x1 = (fingersCoordinates[self.positionFingersMCP["index"]][1] - fingersCoordinates[self.wristPosition][1])
        y1 = (fingersCoordinates[self.positionFingersMCP["index"]][2] - fingersCoordinates[self.wristPosition][2])
        z1 = (fingersCoordinates[self.positionFingersMCP["index"]][3] - fingersCoordinates[self.wristPosition][3])

        x2 = (fingersCoordinates[self.positionFingersMCP["pinky"]][1] - fingersCoordinates[self.wristPosition][1])
        y2 = (fingersCoordinates[self.positionFingersMCP["pinky"]][2] - fingersCoordinates[self.wristPosition][2])
        z2 = (fingersCoordinates[self.positionFingersMCP["pinky"]][3] - fingersCoordinates[self.wristPosition][3])

        vector1 = np.array([x1, y1, z1])
        vector2 = np.array([x2, y2, z2])

        normalVector = np.cross(vector2, vector1, axisa=- 1, axisb=- 1, axisc=- 1, axis=None)

        return normalVector
    
    def get_fingers_angle(self, fingerVector, normalVector):
        self.fingersAngles = []
        for finger in range(0, len(fingerVector)):
            if(fingerVector[finger][1] >= 0):
                result = 1
            else:
                normalizeFingerVector = fingerVector[finger] / np.linalg.norm(fingerVector[finger])
                normalizeNormalVector = normalVector / np.linalg.norm(normalVector)
                moduleFingerVector = math.sqrt(normalizeFingerVector[0]**2 + normalizeFingerVector[1]**2 + normalizeFingerVector[2]**2)
                moduleNormalVector = math.sqrt(normalizeNormalVector[0]**2 + normalizeNormalVector[1]**2 + normalizeNormalVector[2]**2)
                result = np.dot(normalizeNormalVector, normalizeFingerVector) / (moduleFingerVector * moduleNormalVector)
            angle_degrees = math.acos(result) * 180 / math.pi
            self.fingersAngles.append(angle_degrees)
        return self.fingersAngles
