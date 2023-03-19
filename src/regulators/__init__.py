import math
import numpy as np
from abc import ABC, abstractclassmethod

MAX_ANGLE = math.radians(20)

class Mode(ABC):
    @abstractclassmethod
    def regul(self):
        pass
    def average(self, list):
        return sum(list)/len(list)
    def limit(self, control_signal):
        return np.clip(control_signal, -MAX_ANGLE, MAX_ANGLE)
