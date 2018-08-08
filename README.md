# Self-Driving Car Engineer Nanodegree Program
Model Predictive Controller Project


### Project Overview
This is a project of the [Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) Term 2. <br>

The main goal of this project is to implement Model Predictive Control to drive the car around the track. In this project the cross track error is not given. Instead it will be calculated. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

### Model Predictive Control

The implememted vehicle model used in this project is a kinematic bicycle model. The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), velocity, the cross-track error and psi error (epsi). Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:
```
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

### Timestep Length and Elapsed Duration (N & dt)

The values I finally chose in my project for N and dt are 7 and 0.1, respectively. 

Originally I used the Udacity's suggested N = 10 and dt = 0.1 for the project. I have adjusted either N or dt (by small amounts) and observed different behaviors, such as 10 / 0.5, 9 / 0.125, 6 / 0.35, 7 / 0.118 and many others.

### Polynomial Fitting and MPC Preprocessing

Before polynomial fitting the path returned from the simulator, we preprocess the points to move them to the origin (x, y) and also rotate the angle to align the car orientation:
```
          for (int i = 0; i < ptsx.size(); i++) {
            //shift car reference angle to 90 degrees
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            double minus_psi = 0 - psi;

            ptsx[i] = (shift_x * cos(minus_psi) - shift_y * sin(minus_psi));
            ptsy[i] = (shift_x * sin(minus_psi) + shift_y * cos(minus_psi));
          }
```

After this preprocessing, the helper function polyfit is called for polynomial fitting. 


### Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

The implemented project set 100ms for latency to mimic real driving conditions.
```
          // Handle latency
          // predict next state to avoid Latency problems
          // Latency of .1 seconds (100 ms)
          double latency_dt = 0.1;
          const double Lf = 2.67;
          double x1=0, y1=0,  psi1=0, v1=v, cte1=cte, epsi1=epsi;

          x1 += v * cos(psi) * latency_dt;
          y1 += v * sin(psi) * latency_dt;
          //steer_value is negative
          psi1 += - v * steer_value / Lf * latency_dt;
          v1 += v + throttle_value * latency_dt;
          cte1 += v * sin(epsi) * latency_dt;
          //steer_value is negative
          epsi1 += - v * steer_value / Lf * latency_dt;
```

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
