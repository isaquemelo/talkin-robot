#!/usr/bin/env python3
#import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
# from simple_pid import PID
# import json
import time

# from calibrated_consts import black_line_following
import paho.mqtt.client as mqtt
from struct import *
# from threading import Thread

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class PipeLineRobot:
    # ev3.Sound.speak("Robot started...")

    # CRUCIAL METHODS
    # DO NOT UPDATE
    def __init__(self):
        self.DEFAULT_SPEED = 400

        self.pipe = None

        # define sensors
        # self.gyroscope_sensor = ev3.GyroSensor('in2')
        # self.reset_gyroscope()
        # self.gyro_value = 0
        #
        # self.color_sensors = (ev3.ColorSensor('in1'), ev3.ColorSensor('in4'))
        # self.dict_colors = {
        #     0: 'Undefined',
        #     1: 'Black',
        #     2: 'Blue',
        #     3: 'Green',
        #     4: 'Yellow',
        #     5: 'Red',
        #     6: 'White',
        #     7: 'Brown'
        # }
        #
        # self.ultrasonic_sensors = {"right": 10, "left": 10}
        # self.infrared_sensors = {"diagonal_top": ev3.InfraredSensor('in3'), "left": 10, "front": 10}
        #
        # # define motors
        # self.motors = Duo(ev3.LargeMotor('outB'), ev3.LargeMotor('outD'))
        # self.motors.left.polarity = "inversed"
        # self.motors.right.polarity = "inversed"
        #
        # self.handler = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outA'))
        # self.handler.left.run_forever(speed_sp=-150)

        # define status
        self.historic = [""]

        # watter server settings
        self.has_pipe = False
        self.current_pipe_size = 10  # [10, 15, 20]
        self.status = "findingHole"  # [placingPipe, findingHole, waitingForMasterRescue, pipeTransfering, initialPositionReset]

        # sever settings
        self.local_ip = "localhost"
        self.receiver = mqtt.Client()
        self.receiver.connect(self.local_ip, 1883, 60)
        self.receiver.on_connect = self.receiver_on_client_connect
        self.receiver.on_message = self.receiver_on_client_message
        self.receiver.loop_start()

        self.external_ip = "192.168.0.1"
        self.publisher = mqtt.Client()
        self.publisher.connect(self.external_ip, 1883, 60)
        self.publisher.on_connect = self.publisher_on_client_connect
        # self.publisher.on_message = self.publisher_on_client_message
        self.publisher.on_publish = self.publisher_on_client_publish
        self.publisher.loop_start()


    def publisher_on_client_publish(self, client, userdata, result):  # create function for callback
        # print("data published")
        pass

    def publisher_on_client_message(self, client, userdata, message):
        print("message received: ", end="")
        payload = unpack("i", message.payload)
        print(payload[0])

    def publisher_on_client_connect(self, client, userdata, flags, rc):
        print("The bluetooth bricks are connected with result code", str(rc))
        client.subscribe("topic/status")

    def receiver_on_client_message(self, client, userdata, message):
        print("message received: ", end="")
        payload = unpack("i", message.payload)

        # self.ultrasonic_sensors['left'] = payload[0]
        # self.ultrasonic_sensors['right'] = payload[1]
        # self.infrared_sensors['front'] = payload[2]
        # self.infrared_sensors['left'] = payload[3]

        print(payload)

    def receiver_on_client_connect(self, client, userdata, flags, rc):
        print("The bluetooth bricks are connected with result code", str(rc))
        client.subscribe("topic/PipeLineRobot")

    def publish_data(self):
        message = pack("i", self.status)
        self.publisher.publish("topic/status", message, qos=0)
        print("dado publicado:", unpack("i", message))


robot = PipeLineRobot()
while True:
    msg = input()
    robot.status = int(msg)
    robot.publish_data()