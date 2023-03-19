from threading import Thread
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
    q = Queue()
    plotter = Plotter(q)
    regulmanager = Regulmanager(plotter, q, HOST)
    #regul = Regul(plotter, q, HOST)
    #regul_thread = Thread(target = regul.run)
    regulmanager.start()
    #plotter.start()
    plt.show()


    #plotter.join()

main()

