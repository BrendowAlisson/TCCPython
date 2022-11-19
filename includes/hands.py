import mediapipe as mp
import numpy as np
from includes import vectors

vector = vectors.Vector()

class Hand:
    def __init__(self):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils

    def get_hand_coordinates_in_video(self, frameInput, frameOutput, x, y):
        self.result = self.hands.process(frameInput)
        self.fingersCoordinates = []
        if self.result.multi_hand_landmarks:
            for handCoordinates in self.result.multi_hand_landmarks:
                for id, coordinates in enumerate(handCoordinates.landmark):
                    coordinatex = int(coordinates.x * y)
                    coordinatey = int(coordinates.y * x)
                    self.fingersCoordinates.append([id, coordinates.x, coordinates.y, coordinates.z])
                self.mpDraw.draw_landmarks(frameOutput, handCoordinates, self.mpHands.HAND_CONNECTIONS)

    def get_fingers_angle(self):
        try:
            fingersCoordinatesVector = vector.create_finger_coordinates_vector(self.fingersCoordinates)
            normalVector = vector.create_normal_vector(self.fingersCoordinates)
            fingerAngles = vector.get_fingers_angle(fingersCoordinatesVector, normalVector)
            print(vector.fingersAngles)
        except:
            print("Waiting hand")
            pass