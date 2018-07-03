# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

This project is about the Model Predictive Controller (MPC) that needs to be implemented to be able to drive safely around the simulated track inside the [Udacity self driving car simulator](https://github.com/udacity/self-driving-car-sim). There is an artificial 100ms delay in the control path, that should add a more realistic scenario. This is similar to real world actuators that react delayed to given commands.


## The model

The MPC controller uses a simple kinematic model that includes the following state variables: vehicles coordinates `(x,y)`, velocity `(v)`, the crosstrack error `(cte)`, the orientation `(psi)` as well as the orientation error `(epsi)`.
The control inputs are acceleration `(a)` and the steering angle `delta`.

The model of the vehicle is defined as:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt;
v[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psi_des + v[t]/Lf * delta[t] * dt;
```

t1 is the timestep after t. This is used for the predictions of the mpc and defined based on the user input for timestep length `N` and elapsed duration `dt`.
Lf is a measure for the distance between the front of the vehicle and its center of gravity. 

I chose the values `N=10` and `dt=0.1` that worked well for my model.


## Processing Pipeline
After N and dt is defined, the first thing I did was to transform the given waypoints into the vehicle coordinates (at vehicles origin, with zero orientation angle).

```
ptsx_vehicle[i] = x * cos(-psi) - y * sin(-psi)
ptsy_vehicle[i] = x * sin(-psi) + y * cos(-psi)
```

After this step, a polynomial (in this case of order 3) fit on the waypoints in vehicle coordinates is performed.
Using the polynomial coefficients and the vehicle state, a solver (we use IPOPT) is used to find the best trajectory fit. This prediction takes in consideration the kinematic model as well as constraints that can be used to further improve the fit by penalizing unwanted behavoir (Example: minimizing the crosstrack error, or high speed at high steering angle, etc).

At each iteration a new trajectory is calculated of which we can use the first predicted value pair (acceleration and steering angle) to control the vehicle. (The sunsequent values at a later timestep are less accurate due to the uncertainty that adds up with each timestep of a give state).

## Results

With this controller the vehicle is able to drive safely around the simulated track reaching the target speed of 100 mph. Without the added delay of 100ms higher speeds are possible. You can find the [video here](assets/mpc.m4v). The yellow line describes the waypoints, and the green is the predicted path give the state that the vehicle follows.
To deal with the 100 ms delay I didn't have to do anything in addition. The model is very robust against delays at this scale. One way to mitigate the effects would be to predict the vehicle state at the future time at +100 ms, and use this state as the vehicle state as input to the model. This would let the model do the calculations from the predicted future state. 



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
