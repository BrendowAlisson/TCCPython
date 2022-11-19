from paho.mqtt import client as mqtt_client
import random
import json

class MQTT:
    def __init__(self, broker='broker.hivemq.com', port= 1883, topic="/topic/esp32/mao"):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.client = self.connect_mqtt()

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print('Connection successful!')
            else:
                print(f'error: {rc}')
        client = mqtt_client.Client(self.client_id)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client

    def publish_message(self, payload):
        try:
            msg = {"servoAnglePinky":int(payload[0]),"servoAngleRing":int(payload[1]), "servoAngleMiddle":int(payload[2]),"servoAngleIndex":int(payload[3]),"servoAngleThumb":int(payload[4])}
            json_object = json.dumps(msg, indent = 4)
            self.client.publish(self.topic, json_object, 0)
            print(msg)
        except:
            pass