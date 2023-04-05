import math
import numpy as np
import mycartopy
from states import State
from logger import Logger
from trajectory import Trajectory, Point
from plotter import Plotter
from time import time, sleep
from dataclasses import dataclass
from controllers import Controller
from data import Data
from regulators import Mode
from communication.uart import Connection

CONTROLLER_PERIOD = 0.2

class Regul_not_sim(Mode):
    def __init__(self, HOST):
        self.HOST = HOST
        self.last_lat = 0.0
        self.last_lng = 0.0
        self.last_yaw = 0.0

    def is_calibrated(self, rc):
        rc.request_states()
        sleep(0.05)
        states = rc.receive_states()
        states = states[:-1]
        print(states)
    
        for state in states:
            if math.isnan(state):
                return False
        return True
    
    def wait_calibrated(self, rc, plotter):
        msg = f'No GPS fix: waiting'
        i = 0
        while not self.is_calibrated(rc):
            plotter.update_status(msg+ '.'*i)
            i +=1
            i = i % 4
            sleep(0.5)
        plotter.update_status("Calibrated")

        sleep(0.2)
        rc.request_states()
        sleep(0.15)
        s = rc.receive_states()
        print(s)
        self.last_lng, self.last_lat, _, _ = s

    def process_response(self, response, states):
        if response is None:
            return
        if math.isnan(response[0]) or math.isnan(response[1]):
            return

        lng, lat, v, _ = response
        states.v = v

        # does the new coordinates differ from the last?
        if lng != self.last_lng and lat != self.last_lat:
            new_x, new_y = mycartopy.to_cartesian_coordinates(lng, lat)

            last_x, last_y = mycartopy.to_cartesian_coordinates(self.last_lng, self.last_lat)
            states.yaw = np.arctan2(new_y - last_y , new_x - last_x)
            #states.yaw = np.arctan2(new_y - states.y, new_x - states.x)

            # update states
            states.x = new_x
            states.y = new_y
            self.last_lng = lng
            self.last_lat = lat
        #if yaw != self.last_yaw:
            #states.yaw = yaw


    def regul(self, log, controller, t, plotter , v_ref, event):
        with Connection(self.HOST) as rc:
            self.wait_calibrated(rc, plotter)
            sleep(1.0)

            # first point is our starting point
            p = t.next_point()
            self.last_yaw = p.orientation

            # initialize states and attr
            last_x, last_y = mycartopy.to_cartesian_coordinates(self.last_lng, self.last_lat)
            state = State(last_x, last_y, p.orientation, v = 0.0, t = CONTROLLER_PERIOD)

            error = [0]
            last_target_idx, _ = t.calc_target(state)

            sleep(1.0)
            # j, number of iterations
            j = 0
            t0 = time()
            while True:
                if event.is_set():
                    break
                t1 = time()
                response = rc.get_states()

                self.process_response(response, state)
                current_target_idx, error_front_axle = t.calc_target(state)

                # is the new point already passed?
                if last_target_idx >= current_target_idx:
                    current_target_idx = last_target_idx

                dist_error = t.calc_dist_sign(error_front_axle, state.yaw, current_target_idx)
                heading_ref = t.get_heading(state.yaw, current_target_idx)
                theta_e = heading_ref - state.yaw

                control_signal = controller.calculate_output(dist_error, theta_e)
                control_signal = self.limit(control_signal)

                # wait a bit to not overload car
                sleep(0.05)
                rc.writeouput(control_signal, v_ref)
        
                state.update_state_space(control_signal, state.v)

                target_x, target_y = t.get_targets(current_target_idx)

                timestamp = time() - t0

                # update plots
                plotter.update(state, heading_ref, v_ref ,timestamp)
                plotter.set_target(target_x, target_y)

                # log magnitude of state error
                state_error_magnitude = np.linalg.norm([theta_e, dist_error])
                error.append(state_error_magnitude)
        
                j += 1
                last_target_idx = current_target_idx

                if t.is_done(current_target_idx):
                    break

                duration = CONTROLLER_PERIOD - (time() - t1)

                if duration > 0:
                    sleep(duration)
                else:
                    print("Lagging behind...")
                    sleep(0.05)

        
            print(f'Goal Has been reached successfully in {j} iterations\nTime {round(time() - t0,1)}s, error {round(self.average(error),3)}')
        
            log.close_files()
