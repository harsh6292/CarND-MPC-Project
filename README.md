# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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

## Explanation


#### MPC Model

The project uses Global Kinematic Model to model a vehicle in motion. This model contains state as (x, y, psi, v, cte, epsi). These states are, in order, the current location of car (x, y), the heading or orientation of the car (psi), the velocity of the car (v), the Cross Track Error (cte - error between the actual y position of car and the expected y position of the car) and change in the orientation of the car (epsi).

This model also takes into account the actuator values of the car (values that helps the car to go into motion). These are the throttle applied (a) and the steering angle applied to the car (delta).

The Global Kinematic Model uses the below equations to model the vehicle motion and predict its next state.

```
x(t+1) = x(t) + v(t) * cos( psi(t) ) * dt

y(t+1) = y(t) + v(t) * sin( psi(t) ) * dt

psi(t+1) = psi(t) + (v(t)/L_f ) * delta(t) * dt

v(t+1) = v(t) + a(t) * dt

cte(t+1) = cte(t) + v(t) * sin( epsi(t) ) * dt

epsi(t+1) = epsi(t) + (v(t)/Lf) * delta(t) * dt

where dt is the time difference between next state and current state
```


#### Timestep Length and Elapsed Duration (N & dt)
The timestep length (N) and elapsed duration (dt) together calculates the total time the MPC should model and estimate the lowest cost path for N*dt time. I initially chose N as 10 and dt 0.1 for a total 1 second to model. It turned out to be on the conservative side and model did not perform as expected. The vehicle started to wobble around after a few seconds into the loop. Consecutively, I reduced the vehicle reference velocity to 30mph and increased the timestep length (N) to 15 steps. This made the vehicle to go around the loop without oscillating too much.


#### Polynomial Fitting and MPC Preprocessing
I first preprocessed the waypoints received from the simulator before fitting the polynomial. The waypoints received from simulator were first converted into vehicle coordinate system with current position of vehicle mapped to origin and heading of the vehicle aligned to x-axis or in the direction of the vehicle.

The vehicle state, actuators and waypoints were then updated based on the equation described above and taking into account latency (described next) before being passed to MPC procedure.

#### Model Predictive Control with Latency
To handle the 100ms latency into the Kinematic model, I used the above equations with dt as 0.1 second to calculate the position (x, y), velocity (v), cross track error and orientation error (cte, delta). This new state is considered as the current state by MPC and then used by MPC to further predict and calculate the actuators value (steering angle and throttle) in order for vehicle to minimize the various errors (cte, delta etc).
