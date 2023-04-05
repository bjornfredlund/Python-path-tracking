import math
import numpy as np
from states import State
from logger import Logger
from trajectory import Trajectory, Point
from plotter import Plotter
from time import time, sleep
from dataclasses import dataclass
from controllers import Controller
from data import Data
from regulators import Mode

CONTROLLER_PERIOD = 0.15


class Regul_sim(Mode):
    def __init__(self):
        pass
    def regul(self, log, controller, t, plotter , v_ref, event):
        # first point is our starting point
        p = t.next_point()
 
        state = State(p.x,p.y, p.orientation, v = v_ref, t = CONTROLLER_PERIOD )
        error = [0]
        last_target_idx, _ = t.calc_target(state)

        # j, number of iterations
        j = 0
        t0 = time()
        while True:
            if event.is_set():
                break
            t1 = time()

            current_target_idx, error_front_axle = t.calc_target(state)

            # is the new point already passed?
            if last_target_idx >= current_target_idx:
                current_target_idx = last_target_idx

            dist_error = t.calc_dist_sign(error_front_axle, state.yaw, current_target_idx)
            heading_ref = t.get_heading(state.yaw, current_target_idx)
            theta_e = heading_ref - state.yaw

            control_signal = controller.calculate_output(dist_error, theta_e)
            control_signal = self.limit(control_signal)

            state.update_state_space(control_signal, v_ref)
    
            timestamp = time() - t0

            target_x, target_y = t.get_targets(current_target_idx)

            plotter.update(state, heading_ref, v_ref, timestamp)
            plotter.set_target(target_x, target_y)

            # log magnitude of state error
            state_error_magnitude = np.linalg.norm([theta_e, dist_error])
            error.append(state_error_magnitude)
    
            j +=1
            last_target_idx = current_target_idx

            if t.is_done(current_target_idx):
                break

            duration = CONTROLLER_PERIOD - (time() - t1)

            if duration > 0:
                sleep(duration)
            else:
                print("Lagging behind...")
    
        print(f'Goal Has been reached successfully in {j} iterations\nTime {round(j*CONTROLLER_PERIOD,1)}s, average error {round(self.average(error),3)}')
    
        log.close_files()

