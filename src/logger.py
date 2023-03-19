import os
from contextlib import ExitStack

path_file = "path_taken.txt"
state_file = "state_log.txt"
control_file = "control_log.txt"
error_file = "error_log.txt"
trajectory_file = "trajectory.txt"
orientation_file = "orientation.txt"

PATH = "log/"


"""
Logs data about the states, trajectory, and control signals in the folder ../log/
"""
class Logger:
    def __init__(self):
        files = [path_file,state_file,control_file,error_file, trajectory_file, orientation_file]
        if not os.path.exists(PATH):
            os.makedirs(PATH)
        self.files = [PATH + f for f in files]
        # dictionary of file descriptors
        self.fds = {path: open(path,'w') for path in self.files}

    def log_vec(self,vec,file_name):
        try:
            for element in vec:
                self.fds[file_name].write(str(element) + " ")
            self.fds[file_name].write("\n")
        except:
            print(f'Something went wrong when trying to write to file: {self.fds[file_name]}')
            self.close_files()

    def log_trajectory(self, coords):
        name = PATH + trajectory_file
        x_v = []
        y_v = []
        for x,y in coords:
            x_v.append(x)
            y_v.append(y)
        self.log_vec(x_v, name)
        self.log_vec(y_v, name)

    def log_pos(self,positions):
        name = PATH + path_file
        self.log_vec(positions,name)
    def log_orientation(self, orientation):
        name = PATH + orientation_file
        self.log_vec(orientation, name)

    def log_state(self,state):
        name = PATH + state_file
        self.log_vec(state,name)
    
    def log_error(self,error):
        name = PATH + error_file
        self.log_vec(error,name)

    def log_control_signal(self,u):
        name = PATH + control_file
        self.log_vec(u,name)

    def close_files(self):
        for path in self.fds:
            self.fds[path].close()
