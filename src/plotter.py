import numpy as np
import math
import itertools
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
import mycartopy
from time import time
from generate_sample_trajectory import generate_sine, generate_tanh
from trajectory import Trajectory
from matplotlib.widgets import Button, RadioButtons
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from controllers import Stanley, Lqr
from dataclasses import replace
from data import Data
from communication.uart import locate


TIME_WINDOW = 5 # seconds
OFFSET = 1
ADJUST_SCALE = 0.5


class Plotter:
    def __init__(self, queue):
        self.q = queue # queue to communicate with regulmanager
        self.d = Data() # data to share with regulmanager
        self.fig = plt.figure(figsize=(11,8)) # open matplotlib figure
        self.timestamp = 0

        # telemetry lists
        self.state_history = []
        self.control_signal_history = []
        self.time_history = [0]
        self.speed_history = [0]
        self.speed_reference_history = [0]
        self.orientation_history = [0]
        self.orientation_reference_history = [0]

        # Velocity plot & reference
        self.velocity_plot = self.fig.add_subplot(321)
        self.status_text = self.velocity_plot.text(0.0, 1.02, "", fontsize = 'x-large')
        self.velocity_plot.set_ylabel('speed (m/s)')
        self.velocity_artist = Line2D(self.time_history, self.speed_history, color = 'C0', label = 'Velocity')
        self.velocity_artist_reference = Line2D(self.time_history, self.speed_reference_history, color = 'Orange',label= 'Reference')
        #self.x_control_artists.legend(loc = "best")
        self.velocity_plot.add_line(self.velocity_artist)
        self.velocity_plot.add_line(self.velocity_artist_reference)
        self.velocity_plot.legend(loc = "best")


        self.orientation_plot = self.fig.add_subplot(322)
        self.orientation_plot.set_ylabel('Orientation (' + chr(176) + ')')
        self.orientation_artist = Line2D(self.time_history, self.orientation_history, color = 'C0', label = 'Orientation')
        self.orientation_artist_reference = Line2D(self.time_history, self.orientation_reference_history, color = 'Orange',label= 'Reference')
        self.orientation_plot.add_line(self.orientation_artist)
        self.orientation_plot.set_ylim(- math.degrees(math.pi) - OFFSET, math.degrees(math.pi)+OFFSET)
        self.orientation_plot.add_line(self.orientation_artist_reference)
        self.orientation_plot.legend(loc = "best")

        self.lng_pos = []
        self.lat_pos = []

        # your favourite wild strawberry spot
        self.lat = None
        self.lng = None
        if self.lng is None:
            print("Add your coordinates")
            self.q.put(None)
            return 

        self.ax = mycartopy.osm_image(self.lng, self.lat, self.fig)

        # widget axes
        cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.controller_ax = self.fig.add_axes([0.70, 0.45, 0.2, 0.10])
        self.controller_ax.set_title('Controller type')
        self.ax_go = self.fig.add_axes([0.70, 0.25, 0.2, 0.06])
        self.ax_mode = self.fig.add_axes([0.7, 0.35, 0.2, 0.10])
        self.ax_load = self.fig.add_axes([0.70, 0.15, 0.2, 0.06])

        # create widgets
        self.go_button = Button(self.ax_go, 'Go', color = 'red')
        self.radio_mode = RadioButtons(self.ax_mode, ('Sim', 'Not sim'))
        self.load_button = Button(self.ax_load, 'Load trajectory')
        self.radio_controller = RadioButtons(self.controller_ax, ('Stanley', 'LQR'))

        # attach actionlistener
        self.load_button.on_clicked(self.load_custom_trajectory)
        self.go_button.on_clicked(self.go)
        self.radio_mode.on_clicked(self.wait_located)

        self.pos_artist, = self.ax.plot([], [], transform = ccrs.PlateCarree())
        self.trajectory_artist, = self.ax.plot([], [], 'o',transform = ccrs.PlateCarree(), label='Waypoints')


        self.ax.add_line(self.pos_artist)
        self.ref_artist, = self.ax.plot([],[],"xr", label = 'Target', transform=ccrs.PlateCarree())
        self.ax.legend(loc = "best")
        self.ani = FuncAnimation(self.fig, self.on_frame, blit=False, save_count=2000, interval = 100)


    def wait_located(self, label):
        if label == 'Sim':
            return
        self.clear_plots()
        self.d.trajectory = None
        self.lng, self.lat = locate()
        self.ax.plot(self.lng,self.lat, color='red', marker='x', ms=6, mew=2, transform=ccrs.PlateCarree())

    def go(self, event):

        self.d.mode = self.radio_mode.value_selected
        self.d.controller = self.radio_controller.value_selected
        if self.d.is_ready():
            self.q.put(self.d)
        else:
            print("Please select all options")

    def update_status(self, text):
        self.status_text.set_text(text)

    def clear_pos(self):
        self.lng_pos.clear()
        self.lat_pos.clear()
        self.trajectory_artist.set_data([],[])

    def clear_plots(self):
        self.clear_pos()
        self.clear_telemetry()

    def clear_telemetry(self):
        self.orientation_history.clear()
        self.orientation_reference_history.clear()
        self.time_history.clear()
        self.speed_history.clear()
        self.speed_reference_history.clear()

    def load_custom_trajectory(self, event):
        self.clear_plots()

        # generate
        #x_coords,y_coords = generate_sine()
        x_coords,y_coords = generate_tanh()

        lng_m, lat_m = self.lng, self.lat

        # fetch location and transform
        cx, cy = mycartopy.to_cartesian_coordinates(lng_m,lat_m)

        # add to generated trajectory coordinates system
        x_new = [cx + x for x in x_coords]
        y_new = [cy + y for y in y_coords]

        # transform for pos plot
        coords = [mycartopy.to_geographic_coordinates(lng,lat) for lng, lat in zip(x_new,y_new)]

        # unzip new geographic coordinates
        lat_new, lng_new = list(zip(*coords))
        self.trajectory_artist.set_data(lng_new,lat_new)

        self.go_button.color = 'green'

        self.d.trajectory = Trajectory(list(zip(x_new, y_new)))

    def set_target(self, x, y):
        lat, lng = mycartopy.to_geographic_coordinates(x, y)
        self.ref_artist.set_data(lng,lat)

    def on_frame(self, frame):
        self.pos_artist.set_data(self.lng_pos, self.lat_pos)
        self.velocity_artist.set_data(self.time_history, self.speed_history)
        self.orientation_artist.set_data(self.time_history, self.orientation_history)
        self.velocity_artist_reference.set_data(self.time_history, self.speed_reference_history)
        self.orientation_artist_reference.set_data(self.time_history, self.orientation_reference_history)

        # nothing to plot...
        if len(self.time_history) == 0:
            return

        # make plot rolling
        if self.timestamp > self.time_history[0] + TIME_WINDOW:
            self.time_history.pop(0) 
            self.speed_history.pop(0)
            self.speed_reference_history.pop(0)
            self.velocity_plot.set_xlim(self.time_history[0],self.time_history[0]+TIME_WINDOW)

            self.orientation_history.pop(0)
            self.orientation_reference_history.pop(0)
            self.orientation_plot.set_xlim(self.time_history[0], self.time_history[0]+TIME_WINDOW)

        else:
            if len(self.time_history) -1 > 0:
                self.velocity_plot.set_xlim(0, max(self.time_history))
                self.orientation_plot.set_xlim(0, max(self.time_history))


        val = max(self.speed_reference_history + self.speed_history)
        self.velocity_plot.set_ylim(0,val +OFFSET)
        return self.pos_artist,self.velocity_artist_reference, self.velocity_artist, self.orientation_artist, self.orientation_artist_reference, self.ref_artist

    def update(self, state, desired_orientation, v_ref, timestamp):
        x = state.x
        y = state.y
        yaw = state.yaw
        v = state.v
        lat, lng = mycartopy.to_geographic_coordinates(x, y)


        self.update_pos(lat, lng)

        self.update_speed_plot(v,v_ref,timestamp)
        self.update_orientation_plot(yaw, desired_orientation)

    def update_orientation_plot(self, orientation, desired_orientation):
        self.orientation_history.append(math.degrees(orientation))
        self.orientation_reference_history.append(math.degrees(desired_orientation))

    def update_speed_plot(self,actual_speed,v_ref,timestamp):
        self.timestamp = timestamp
        self.time_history.append(timestamp)
        self.speed_history.append(actual_speed)
        self.speed_reference_history.append(v_ref)

        
    def update_pos(self,latitude, longitude):
        self.lng_pos.append(longitude)
        self.lat_pos.append(latitude)

    def on_close(self, event):
        self.q.put(None)
        #self.ani.save('./animation.gif', fps=10)

    def on_click(self, event):
        # did we click on the desired plot?
        if event.inaxes != self.ax:
            return

        self.clear_plots()
        ix, iy = event.xdata, event.ydata

        lng, lat, _ = mycartopy.to_geographic_coordinates_cartopy(x = ix, y = iy)

        lng_path,lat_path = mycartopy.get_path(self.lng, self.lat, lng, lat)

        coords = [mycartopy.to_cartesian_coordinates(lng,lat) for lng, lat in zip(lng_path,lat_path)]

        self.trajectory_artist.set_data(lng_path,lat_path)

        self.go_button.color = 'green'

        # load trajectory into data
        self.d.trajectory = Trajectory(coords)
