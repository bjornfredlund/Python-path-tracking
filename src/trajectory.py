import math
import itertools
from dataclasses import dataclass
from scipy import interpolate
import numpy as np
from collections import namedtuple
from scipy.spatial import distance as scipy_dist
import os

PATH = "trajectories/"

L = 0.3 # wheelbase (m)
DESIRED_PERIOD = 5.0 # meters per point

"""
    Calculates and keeps track of trajectory information, info available for Regul.
"""

class Trajectory:
    def __init__(self, coords):
        self.coords = coords

        # contains all coordinates of path as point objects
        self.point_list = []
        self.current_point = 0

        self.point_list = self.generate_points()
        self.point_list = self.pad_points()
        self.point_list = self.smoothen_path()
        self.length = self.calculate_length()
        self.calculate_orientation()
       
        print(f"Trajectory succefully loaded!\nTrajectory contains {self.number_of_points} points and is {round(self.length,2)}m long")

    def pairwise(self, iterable):
        "s -> (s0,s1), (s1,s2), (s2, s3), ..."
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)

    def calculate_length(self):
        dist = 0
        for prev, current in self.pairwise(self.point_list):
            dist += np.hypot(current.x - prev.x, current.y - prev.y)
        return dist


    def smoothen_path(self):
        # path_length (m): 1 point per meter sufficient trajectory accuracy
        path_length = int(self.calculate_length())

        x, y = zip(*self.coords)
        tck, _ = interpolate.splprep([x, y],s = 0.0)
        x_new, y_new = interpolate.splev(np.linspace(0, 1, path_length) ,tck)

        self.coords = [(x, y) for x, y in zip(x_new, y_new)]

        return self.generate_points()


    def pad_points(self):
        padded_x = []
        padded_y = []
        for prev, current in self.pairwise(self.point_list):
            dist = np.hypot(current.x - prev.x, current.y - prev.y)
            n = int(dist / DESIRED_PERIOD)
            # distance between points smaller than DESIRED_PERIOD
            if n < 1:
                padded_x.append(prev.x)
                padded_y.append(prev.y)
                continue
            padded_x += np.linspace(prev.x, current.x, n, endpoint=False).tolist()
            padded_y += np.linspace(prev.y, current.y, n, endpoint=False).tolist()

        x, y = zip(*self.coords)

        # manually add last point
        padded_x.append(x[-1])
        padded_y.append(y[-1])

        self.coords = [(x, y) for x, y in zip(padded_x, padded_y)]

        return self.generate_points()

    def generate_points(self):
        points = []
        for x, y, in self.coords:
            p = Point(x,y)
            points.append(p)
        self.number_of_points = len(points)
        return points


    def calculate_orientation(self):
        p = self.next_point()
        p_next = self.next_point()

        first_angle = math.atan2(p_next.y - p.y, p_next.x - p.x)

        p.orientation = first_angle

        p_prev = p
        p = p_next
        p_next = self.next_point()

        while p_next != None and p != None:
            theta = math.atan2(p_next.y - p_prev.y, p_next.x - p_prev.x)
            p.orientation = theta

            p_prev = p
            p = p_next
            p_next = self.next_point()

        if self.number_of_points > 2:
            last_angle = math.atan2(p.y - p_prev.y, p.x - p_prev.x)
            p.orientation = last_angle


    def calc_dist_sign(self, dist_error, current_yaw, target_idx):
        point_of_interest = self.point_list[target_idx]
        target_yaw = np.arctan2(self.fy - point_of_interest.y, self.fx - point_of_interest.x) - current_yaw

        if target_yaw > 0.0:
            dist_error = - dist_error
        return dist_error

    def get_heading(self, yaw, target_idx):
        return self.point_list[target_idx].orientation
    
    def is_done(self, target_idx):
        return target_idx >= len(self.point_list) - 1

    def get_targets(self, target_idx):
        p = self.point_list[target_idx]
        return p.x, p.y
    def calc_target(self, state):
        # Calc front axle position
        self.fx = state.x + L * np.cos(state.yaw)
        self.fy = state.y + L * np.sin(state.yaw)

        # index of point closest to car wrt front axle 
        target_idx = scipy_dist.cdist([(self.fx, self.fy)], self.coords).argmin()

        p_closest = self.point_list[target_idx]

        # distance to closest point
        error_front_axle = np.hypot(self.fy - p_closest.y, self.fx - p_closest.x)

        return target_idx, error_front_axle


    def next_point(self):
        if self.current_point < self.number_of_points:
            point_pos = self.current_point
            point = self.point_list[point_pos]
            self.current_point += 1 
            return point
        else:
            self.current_point = 0
            return None

# helper container
@dataclass
class Point:
    x: float
    y: float
    orientation: float = 0.0

    def __iter__(self):
        return (self.x, self.y)
