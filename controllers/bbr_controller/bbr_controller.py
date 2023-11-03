# Author: Jacob Turner
from controller import Robot
from datetime import datetime
import math
import numpy as np

# basic architecture taken from labs <insert labs>
class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = 32 # milliseconds
        self.max_speed = 1   # meters per second
        
        # motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        # set velocity initially to 0
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        # set the velocoties to 0
        self.velocity_left = 0
        self.velocity_right = 0
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag for if the light was on or off
        self.flag_light = 0
    # end contructor
    
    def run_robot():
        pass
    # end run_robot
# enc class Controller

if __name__ == "__main__":
    behavioural = Robot()
    controller = Controller(behavioural)
    controller.run_robot()