import cv2
from includes import hands

wCam, hCam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

myHand = hands.Hand()

def main():
    while True:
        sucess, img = cap.read()
        x, y, c = img.shape
        frameBGR = cv2.flip(img, 1)
        frameRGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)
        myHand.get_hand_coordinates_in_video(frameRGB, frameBGR, x, y)
        myHand.get_fingers_angle()
        cv2.imshow('output', frameBGR)

        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == '__main__':
    main()