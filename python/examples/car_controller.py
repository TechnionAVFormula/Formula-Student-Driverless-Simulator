import sys
import os

## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')))

import time
import fsds
import numpy as np


def scale_input(input, input_min=0, input_max=1, output_min=0, output_max=1):
    return ((input - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min


class CarController:
    def __init__(self):
        self.client = fsds.FSDSClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def sendDriveInstruction(self, gas, brakes, steering):
        car_controls = fsds.CarControls()
        car_controls.throttle = scale_input(gas)
        car_controls.brake = scale_input(brakes)
        car_controls.steering = steering
        self.client.setCarControls(car_controls)


if __name__ == '__main__':
    car = CarController()
    while True:
        # car_state = car.client.getCarState()
        # print('car speed (m/s): {0}'.format(car_state.speed))
        car.sendDriveInstruction(0.9, 0, 0.1)
        time.sleep(0.5)
