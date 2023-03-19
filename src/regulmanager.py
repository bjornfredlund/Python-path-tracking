import threading
from dataclasses import dataclass
from trajectory import Trajectory
from logger import Logger
from plotter import Plotter
from controllers import Stanley, Lqr
from data import Data
from regulators.regul_sim import Regul_sim
from regulators.regul_not_sim import Regul_not_sim


class Regulmanager(threading.Thread):
    def __init__(self, plotter, queue, HOST):
        threading.Thread.__init__(self)
        self.plotter = plotter
        self.q = queue 
        self.HOST = HOST

    def process_data(self, data):
        c = None
        m = None

        if data.mode == 'Sim':
            m = Regul_sim()
        elif data.mode == 'Not sim':
            m = Regul_not_sim(self.HOST)

        if data.controller == 'Stanley':
            c = Stanley(v = data.v)
        elif data.controller == 'LQR':
            c = Lqr(v = data.v)

        return m, c, data.trajectory

    def run(self):
        while True:
            # fetch data
            data = self.q.get()

            # if None is in queue -> terminate execution
            if data is None:
                break

            # new log, erases the old logs
            log = Logger()
            #try:

            m, c, t = self.process_data(data)
            m.regul(log, c, t, self.plotter, v_ref = data.v)
            #except:
                #log.close_files()
