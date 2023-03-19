# Python path tracking 
---------
Exploring different path tracking techniques with visualization. 

Can be run as simulation or together with [this](https://github.com/bjornfredlund/ESP32---RC-interface) and [this](https://github.com/bjornfredlund/ESP32---UART-NRF24-station) project.
Uses [Openrouteservice](https://openrouteservice.org/) for path or able to load various test paths.
## To use
----
Specify your favourite coordinates in ```src/plotter.py``` and fetch api key from [Openrouteservice](https://openrouteservice.org/) or load your custom paths with ```src/generate_sample_trajectories.py```
Run the program with:
```
make
``` 
or run ```Python3 src/main.py```


## Modeling
----
#### LQR
Uses the error in distance between the car and closest point on the path, d, and the error in heading θ_{e} (difference between car heading and tangential angle of the closest point).
```math
    \dot{d} = v*sin(θ_{e})
```
```math
    \dot{θ_{e}} = v*(u - c(p)/(1-d*c(p)))
```
where $c(p)$ is the curvature of the path at point p, u is the control signal (steering input) and d as above. System linearized around $c(p)$ = 0 and θ_{e} = 0.

Poor performance with waypoints far from each other.

#### Stanley
Geometric controller with the control signal
```math
    u = θ_{e} + atan(k*d/(Ks*v))
```
where θ_{e}, v and d same as above and k, Ks tuning/softening constants.
