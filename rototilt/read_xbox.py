import pygame
import sys


class Xbox:

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise Exception("No controller detected. Please connect an Xbox controller.")
        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.init_vals()


    def init_vals(self):
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0

    def read_value(self):
        # Process events (required for pygame to update joystick state)
        pygame.event.pump()
        scale = 1e-1
        # Read left stick (axes 0 and 1)
        self.left_x = self.controller.get_axis(0)*scale
        self.left_y = self.controller.get_axis(1)*scale
        
        # Read right stick (axes 2 and 3)
        self.right_x = self.controller.get_axis(2)*scale
        self.right_y = self.controller.get_axis(3)*scale

        return self.left_x, self.left_y, self.right_x, self.right_y
    


    
class DummyXbox:

    def __init__(self):

        self.init_vals()
        
        self.call_counter = 0


    def init_vals(self):
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0

    def read_value(self):

        # todo dummy input
        self.init_vals()


        c1 = 500
        c2 = 1500


        if c1 < self.call_counter:
            self.right_x = -0.1


        self.call_counter += 1


        return self.left_x, self.left_y, self.right_x, self.right_y