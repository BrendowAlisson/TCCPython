import numpy as np
import math
from includes import utils

class Vector:
    def __init__(self):
        self.positionFingersTIP = {"pinky": 20, "ring": 16, "middle": 12, "index": 8, "thumb": 4}
        self.positionFingersMCP = {"pinky": 17, "ring": 13, "middle": 9, "index": 5, "thumb": 2}
        self.orderFingersInVector = {"pinky": 0, "ring": 1, "middle": 2, "index": 3, "thumb": 4}
        self.wristPosition = 0
        self.angleZeroInRadian = 1
    
    def create_finger_coordinates_vector(self, fingersCoordinates):
        vectorCoordinates = []
        for finger in self.positionFingersTIP:
            x = (fingersCoordinates[self.positionFingersTIP[finger]][1] - fingersCoordinates[self.positionFingersMCP[finger]][1])
            y = (fingersCoordinates[self.positionFingersTIP[finger]][2] - fingersCoordinates[self.positionFingersMCP[finger]][2])
            z = (fingersCoordinates[self.positionFingersTIP[finger]][3] - fingersCoordinates[self.positionFingersMCP[finger]][3])
            vectorCoordinates.append(np.array([x, y, z]))
        return vectorCoordinates

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

    def get_angle_between_fingers_palm(self, normalVector, normalizeFingerVector, moduleFingerVector):
        normalizeNormalVector = normalVector / np.linalg.norm(normalVector)
        moduleNormalVector = math.sqrt(normalizeNormalVector[0]**2 + normalizeNormalVector[1]**2 + normalizeNormalVector[2]**2)
        angleRadians = np.dot(normalizeNormalVector, normalizeFingerVector) / (moduleFingerVector * moduleNormalVector)
        return angleRadians

    def get_angle_between_thumb_pinky(self, fingerVector,  normalizeFingerVector, moduleFingerVector):
        normalizePinkyVector = fingerVector[self.orderFingersInVector["pinky"]] / np.linalg.norm(fingerVector[self.orderFingersInVector["pinky"]])
        modulePinkyVector = math.sqrt(normalizePinkyVector[0]**2 + normalizePinkyVector[1]**2 + normalizePinkyVector[2]**2)
        angleRadians = np.dot(normalizePinkyVector, normalizeFingerVector) / (moduleFingerVector * modulePinkyVector)
        return angleRadians

    def get_angle_in_radians(self, fingerVector, normalVector, finger):
        normalizeFingerVector = fingerVector[finger] / np.linalg.norm(fingerVector[finger])
        moduleFingerVector = math.sqrt(normalizeFingerVector[0]**2 + normalizeFingerVector[1]**2 + normalizeFingerVector[2]**2)
        if(utils.is_finger_thumb(finger)):
            angleRadians = self.get_angle_between_thumb_pinky(fingerVector, normalizeFingerVector, moduleFingerVector)
        else:
            angleRadians = self.get_angle_between_fingers_palm(normalVector, normalizeFingerVector, moduleFingerVector)
        return angleRadians

    def transform_radians_in_degrees(self, radians):
        angleDegrees = math.acos(radians) * 180 / math.pi
        return angleDegrees

    def get_fingers_angle(self, fingerVector, normalVector):
        fingersAnglesDegrees = []
        for finger in range(0, len(fingerVector)):
            if(utils.is_finger_thumb(finger)):
                if(utils.is_axis_x_positive(fingerVector[finger][0])):
                    angleRadians = self.angleZeroInRadian
                else:
                    angleRadians = self.get_angle_in_radians(fingerVector, normalVector, finger)
            else:
                if(utils.is_axis_y_positive(fingerVector[finger][1])):
                    angleRadians = self.angleZeroInRadian
                else:
                    angleRadians = self.get_angle_in_radians(fingerVector, normalVector, finger)
            fingersAnglesDegrees.append(self.transform_radians_in_degrees(angleRadians))
        return fingersAnglesDegrees
