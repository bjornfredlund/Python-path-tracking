import math
import numpy as np

L = 0.3 # wheelbase (m)

class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, t=0.2):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.CONTROLLER_PERIOD = t

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
    def update_state_space(self, control_signal, v):
        self.v = v

        self.x += self.CONTROLLER_PERIOD*self.v*math.cos(self.yaw)
        self.y += self.CONTROLLER_PERIOD*self.v*math.sin(self.yaw)

        # new yaw
        theta_dot = self.v*math.tan(control_signal)/L # L wheelbase

        self.yaw += theta_dot*self.CONTROLLER_PERIOD
        #self.yaw = self.normalize_angle(self.yaw)
