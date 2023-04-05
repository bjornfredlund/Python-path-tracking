from threading import Thread, Event
from multiprocessing import Process
from plotter import Plotter
from queue import Queue
#from regulsim import Regul
from dataclasses import dataclass
from controllers import Controller
from regulmanager import Regulmanager
from trajectory import Trajectory
import matplotlib.pyplot as plt


HOST = "COM6"


def main():
    e = Event()
    q = Queue()

    plotter = Plotter(q, e)

    regulmanager = Regulmanager(plotter, q, HOST, e)
    regulmanager.start()

    plt.show()

main()
