import math
import numpy as np
import control
from abc import ABC, abstractclassmethod
from trajectory import Trajectory



class Controller(ABC):
    @abstractclassmethod
    def calculate_output(self):
        pass

class Stanley(Controller):
    def __init__(self, v = 2.5, K = 0.01, Ks = 0.1):
        self.v = v
        self.K = K
        self.Ks = Ks
        
    def calculate_output(self, dist_error, theta_e):
        theta_d = np.arctan2(self.K*dist_error, self.Ks + self.v)

        control_signal = theta_e + theta_d

        return control_signal

class Lqr(Controller):
    def __init__(self, CONTROLLER_PERIOD = 0.15 , v = 2.5):
        self.CONTROLLER_PERIOD = CONTROLLER_PERIOD
        """ define matrices. """
        self.A = np.array([[0, v], [0, 0]])
        self.B = np.array([[0], [v]])

        self.C = np.array([[1, 1 ]])

        self.D = np.array([[0]])
        sys = control.ss(self.A, self.B, self.C, self.D)
        self.Q = np.array([[1, 0],  # Penalize distance error
            [0, 2]])                # Penalize heading error
        self.R = np.array([[25]]) # Penalize steering effort

        self.dsys = sys.sample(self.CONTROLLER_PERIOD)

        self.K, _, _ = control.dlqr(self.dsys, self.Q, self.R)


    def calculate_output(self, dist_error, theta_e):
        error = [dist_error, theta_e]

        u = self.K @ error
        # from ndarray -> float64
        u = u[0]

        return u
