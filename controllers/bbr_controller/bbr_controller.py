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
        # initially False
        self.flag_light = False
    # end contructor
    
    # taken from lab <n>
    def clip_value(self, value, min_max):
        if (value > min_max):
            return min_max
        elif (value < -min_max):
            return -min_max
        return value
    # end clip_value
    def sense_compute_actuate(self):
        pass
    # end sense_compute_actuate
    
    # taken from lab <n>
    def run_robot(self):
        # main loop
        count = 0
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:
            # read ground sensors
            self.inputs = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            
            # change values
            min_gs = 0
            max_gs = 1000
            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs
            
            # save data
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            
            # read proximity sensors
            for i in range(8):
                tmp = self.proximity_sensors[i].getValue()
                # change values
                min_ps = 0
                max_ps = 2400
                if(tmp > max_ps): tmp = max_ps
                if(tmp < min_ps): tmp = min_ps
                # save data
                self.inputs.append((tmp-min_ps)/(max_ps-min_ps))
            # end for
            # smooth filter (average)
            smooth = 30
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_actuate()
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1
        # end while
    # end run_robot
# end class Controller

if __name__ == "__main__":
    behavioural = Robot()
    controller = Controller(behavioural)
    controller.run_robot()