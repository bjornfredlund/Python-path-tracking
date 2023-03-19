import math
from dataclasses import dataclass
from scipy.interpolate import CubicSpline, Akima1DInterpolator
import numpy as np
from collections import namedtuple
from scipy.spatial import distance as scipy_dist
import os

PATH = "trajectories/"

L = 0.3 # wheelbase (m)
THRESHOLD = 1.1

"""
    Calculates and keeps track of trajectory information in memory read from text file, info available for Regul.
"""

class Trajectory:
    def __init__(self, coords):
        #self.file = PATH + f

        self.coords = coords

        # stores x-y references
        self.x_ref = []
        self.y_ref = []
        self.orientation_ref = []


        for lng,lat in coords:
            self.x_ref.append(lng)
            self.y_ref.append(lat)


        # contains all coordinates of path as point objects
        self.point_list = []
        self.current_point = 0

        self.generate_trajectory()
        print(f"Trajectory succefully loaded!\nTrajectory contains {self.trajectory_length} points and is {round(self.length,2)}m long")


    def generate_trajectory(self):
        self.generate_points()
        self.calculate_orientation()

    def generate_points(self):
        for x,y, in zip(self.x_ref, self.y_ref):
            p = Point(x,y)
            self.point_list.append(p)
        self.trajectory_length = len(self.point_list)

    def calculate_orientation(self):
        p = self.next_point()
        p_next = self.next_point()

        first_angle = math.atan2(p_next.y - p.y, p_next.x - p.x)
        self.orientation_ref.append(first_angle)

        p.orientation = first_angle

        p_prev = p
        p = p_next
        p_next = self.next_point()

        length = 0

        # trajectory only contained two points
        if p_next == None:
            hypot = np.hypot(p.x - p_prev.x, p.y - p_prev.y)
            length = hypot
            self.length = hypot


        while p_next != None and p != None:

            hypot = np.hypot(p.x - p_prev.x, p.y - p_prev.y)
            theta = math.atan2(p_next.y - p_prev.y, p_next.x - p_prev.x)
            self.orientation_ref.append(theta)
            p.orientation = theta

            # total length so far
            length += hypot

            p_prev = p
            p = p_next
            p_next = self.next_point()

        self.length = length


        if self.trajectory_length > 2:
            last_angle = math.atan2(p.y - p_prev.y, p.x - p_prev.x)
            self.orientation_ref.append(last_angle)
            p.orientation = last_angle


    def calc_dist_sign(self, dist_error, current_yaw, target_idx):
            target_yaw = np.arctan2(self.fy - self.y_ref[target_idx], self.fx - self.x_ref[target_idx]) - current_yaw

            if target_yaw > 0.0:
                dist_error = - dist_error
            return dist_error

    def get_heading(self, yaw, target_idx):
        return self.point_list[target_idx].orientation
    
    def is_done(self, target_idx):
        return target_idx >= len(self.point_list) - 1

    def get_targets(self, target_idx):
        return self.x_ref[target_idx], self.y_ref[target_idx]
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
        if self.current_point < self.trajectory_length:
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
