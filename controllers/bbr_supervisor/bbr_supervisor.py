"""bbr_supervisor controller.
Author: Jacob Turner
"""
from controller import Supervisor, Field
import sys

# architecture inspired by lab 2
class SupervisorController:
    def __init__(self):
        self.time_step = 32 # milliseconds
        self.flag_light = False # flag for if the light was on or off (False = Off, True = On)
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef('Controller')
        if self.robot_node is None:
            print('No DEF Controller found in world file')
            sys.exit(1)
        self.translation_field = self.robot_node.getField('translation')
        self.rotation_field = self.robot_node.getField('rotation')
        # light fields
        self.spotlight_node = self.supervisor.getFromDef('spotlight')
    # end contructor

    def run(self):
        # set initial robot translation / rotation
        initial_translation = [-0.6859869999894512, -0.6600000001722799, -6.395417340469313e-05]
        initial_rotation = [1.0097124858393808e-05, -9.500172869008803e-06, 0.9999999999038973, 1.63194077882312]
        self.translation_field.setSFVec3f(initial_translation)
        self.rotation_field.setSFVec3f(initial_rotation)
        # set light on (T) or off (F)
        self.spotlight_node.getField('on').setSFBool(False)
        # run until robot is in the reward zone
        # while self.supervisor.step(self.time_step) != -1:
        #     # if self.supervisor.getTime() % 1 < 0.1:
        #         # print('Robot t: ' + str(self.translation_field.getSFVec3f()))
        #         # print('Robot r: ' + str(self.rotation_field.getSFVec3f()))
        #     # todo: check if the robot is in the reward section, then stop!
        #     pass
        # end while
    # end run
# end class SupervisorController

if __name__ == "__main__":
    print("Starting Supervisor Controller")
    supervisor_controller = SupervisorController()
    supervisor_controller.run()
