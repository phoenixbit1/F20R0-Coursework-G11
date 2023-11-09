# Author: Jacob Turner
from controller import Robot
from datetime import datetime
import math
import numpy as np

# basic architecture taken from labs 1,2
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

        # enable light sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)

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
        # flag for what direction to turn
        self.flag_dir = 0
        # flag for if the robot has turned
        self.has_turned = False
        # flag for turning at the end of the line
        self.flag_turn = 0
    # end contructor
    
    # taken from lab 1
    def clip_value(self, value, min_max):
        if (value > min_max):
            return min_max
        elif (value < -min_max):
            return -min_max
        return value
    # end clip_value
    
    def sense_compute_actuate(self):
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # read light sensors
            # for i in range(8):
            #     # display light sensor values
            #     print("Light Sensor {}: {}".format(i, self.inputs[11+i]))
            # end for
            # print("-----------------")

            # check if every light sensor is 1.0
            if(all(x == 1.0 for x in self.inputs[11:18])):
                self.flag_light = False # light is off
            else:
                self.flag_light = True # light is on
                self.flag_dir = 1 # turn according to to light
            # end if
            
            # print(np.max(self.inputs[3:11]))
            # Check for any possible collision
            if(np.max(self.inputs[3:11]) > 0.1):
                # Time
                time = datetime.now()
                print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
            # Turn
            # print(f"flag turn: {self.flag_turn}")
            # print(f"has turned: {self.has_turned}")
            # print(self.inputs[0:3])
            # print(self.inputsPrevious[0:3])
            if(self.flag_turn):
                if(np.min(self.inputs[0:3])< 0.35):
                    self.flag_turn = 0
            else:
                # Check end of line
                if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) > 0.2):
                    print("End of line detected!")
                    self.flag_turn = 1
                    # turn depending on the light
                    if self.has_turned == False:
                        if self.flag_dir == 1: # light is on
                            self.velocity_left = -0.3
                            self.velocity_right = 0.3
                            self.has_turned = True
                        else: # light is off 
                            self.velocity_left = 0.3
                            self.velocity_right = -0.3
                            self.has_turned = True
                else:
                    # Follow the line    
                    if(self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]):
                        self.velocity_left = 0.5
                        self.velocity_right = 1
                    elif(self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
                        self.velocity_left = 1
                        self.velocity_right = 1    
                    elif(self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]):
                        self.velocity_left = 1
                        self.velocity_right = 0.5
     
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
    # end sense_compute_actuate
    
    # mostly taken from lab 1
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
            # read light sensor (this code is inspired by the code above, the code above is from lab 1)
            for i in range(8):
                tmp = self.light_sensors[i].getValue()
                # change the values
                min_ls = 0
                max_ls = 1.0 # max from light sensors
                if(tmp > max_ls): tmp = max_ls
                if(tmp < min_ls): tmp = min_ls
                # save data
                self.inputs.append((tmp-min_ls)/(max_ls-min_ls))
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