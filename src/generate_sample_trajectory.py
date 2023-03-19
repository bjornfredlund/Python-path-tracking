import math
import os
import numpy as np

"""
    Creates sample trajectories for testing 
"""

def generate_cube():

    npoints = 30
    l = 30

    xr = np.linspace(0, l ,npoints)
    yr = []

    for x in xr:
        yr.append(-10*x*x*x + 1.0*x)
        
def split_list(a_list):
    half = len(a_list)//2
    return a_list[:half], a_list[half:]

def generate_tanh():
    npoints = 16
    l = 40 # In m

    #dist = np.linspace(-0.5, l - 0.5,npoints)
    xr = np.linspace(0.0,l,npoints)
    #x_a, x_b = split_list(xr)
    #y_a = 5.0*np.tanh(0.3*x_a - 3)+5.0
    #y_b = - 5.0*np.tanh(0.3*x_b - 11) + 5.0

    yr = 7.0*np.tanh(0.18*xr-4)+7.0
    #yr = yr + - 5.0*np.tanh(0.3*xr[len(xr)//2:]-11) + 5.0
    return xr, yr
        

def generate_circle():

    npoints = 20   # Number of reference points
    r = 8         # Radius of circle to go in
        
    tr = np.linspace(0.0,npoints,npoints, endpoint=True)  
    xr = np.cos(tr*2*np.pi)*r                   # x-coordinates for target points
    yr = np.sin(tr*2*np.pi)*r                   # y-coordinates for target points

    return xr,yr

def generate_sine():
    freq = 0.5
    length = 30
    npoints = 40
    amplitude = 6.0
    xr = np.linspace(0.0,length,npoints)
    yr = amplitude*np.sin(freq*xr)
        
    return xr,yr
