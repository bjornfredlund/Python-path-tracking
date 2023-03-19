from dataclasses import dataclass
from abc import ABC, abstractclassmethod
from trajectory import Trajectory

@dataclass
class Data:
    trajectory: Trajectory = None
    controller: str = None
    mode: str = None
    v: float = 3.0

    def is_ready(self):
        names = [a for a in dir(self) if not a.startswith('__')]

        for attr in names:
            if getattr(self,attr) is None:
                return False

        return True
