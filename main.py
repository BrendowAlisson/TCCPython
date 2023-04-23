import cv2
from includes import hands, mqtt

wCam, hCam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

myHand = hands.Hand()
mqttServer = mqtt.MQTT()

def main():
    mqttServer.client.loop_start()
    while True:
        _, img = cap.read()
        frame = cv2.flip(img, 1)
        myHand.get_hand_coordinates_in_video(frame)
        mqttServer.publish_message(myHand.get_fingers_angle())
        cv2.imshow('output', frame)

        if cv2.waitKey(1) == ord('q'):
            break


if __name__ == '__main__':
    main()