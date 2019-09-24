#!/usr/bin/env python3
import paho.mqtt.client as mqtt
# from time import sleep
# from simple_pid import *
# from threading import Thread
from struct import *


class Robo():
    def __init__(self):
        # earth server settings
        self.has_pipe = False
        self.position = "left"  # what's the side of the requested pipe
        self.current_pipe_size = 10  # [10, 15, 20]
        self.status = "pipeRescue"  # [pipeRescue, waitingForWatterRobotToInitAlign, waitingForWatterRobotPipeRescue, initAlignment]

        # server settings
        self.local_ip = "localhost"
        self.receiver = mqtt.Client()
        self.receiver.connect(self.local_ip, 1883, 60)
        self.receiver.on_connect = self.receiver_on_client_connect
        self.receiver.on_message = self.receiver_on_client_message
        self.receiver.loop_start()

        self.external_ip = "192.168.0.2"
        self.publisher = mqtt.Client()
        self.publisher.connect(self.external_ip, 1883, 60)
        self.publisher.on_connect = self.publisher_on_client_connect
        self.publisher.on_message = self.publisher_on_client_message
        # self.publisher.on_publish = self.publisher_on_client_publish
        self.publisher.loop_start()

    def receiver_on_client_message(self, client, userdata, message):
        # print("info received")
        payload = unpack("i", message.payload)

        print("dado recebido:", payload)

    def receiver_on_client_connect(self, client, userdata, flags, rc):
        print("Receiver - The robots are connected with result code", str(rc))
        client.subscribe("topic/status")

    def publisher_on_client_message(self, client, userdata, message):
        # print("info received")
        payload = unpack("i", message.payload)

        # self.ultrasonic_sensors['left'] = payload[0]
        # self.ultrasonic_sensors['right'] = payload[1]
        # self.infrared_sensors['front'] = payload[2]
        # self.infrared_sensors['left'] = payload[3]

        # print(payload)

    def publisher_on_client_connect(self, client, userdata, flags, rc):
        print("Publisher - The robots are connected with result code", str(rc))
        client.subscribe("topic/PipeLineRobot")


    def publish_data(self):
        message = pack("i", self.status)
        self.publisher.publish("topic/PipeLineRobot", message, qos=0)
        print("dado publicado:", unpack("i", message))


robot = Robo()
while True:
    msg = input()
    robot.status = int(msg)
    robot.publish_data()

