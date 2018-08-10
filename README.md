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

The values I finally chose in my project for N and dt are 10 and 0.1, respectively. 

Firstly I started with the Udacity's suggested N = 10 and dt = 0.1 for the project. Then I adjusted either N or dt (by small amounts) and observed different behaviors, such as 10 / 0.5, 9 / 0.125, 6 / 0.35, 7 / 0.118 , 8 / 0.1, 7 / 0.1 and many others.

I decided to keep N = 10 and dt = 0.1 in my project, because they are good to keep car on trail and have good performance. I tuned other parameters based on these two values to be able to have the car drives smoothly on track.

The value of time horizon ```N * dt``` is the key to get predicted path accurately. Value of ```10 * 0.1``` gives a good prediction in my experiments. Smaller dt will give better accuracy but that will require higher N for the same horizon. However, increase N will result in longer computational time which effectively increase the latency. The most common choice of values is N=10 and dt=0.1.
 
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
          
          // Initial state
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
          double v0 = v;
          double cte0 = coeffs[0];
          double epsi0 = -atan(coeffs[1]);

          // State after delay
          double x1 = x0 + v0 * cos(psi0) * latency_dt;
          double y1 = y0 + v0 * sin(psi0) * latency_dt;
          //steer_value is negative
          double psi1 = psi0 - v0 * steer_value / Lf * latency_dt;
          double v1 = v0 + throttle_value * latency_dt;
          double cte1 = cte0 + v0 * sin(epsi0) * latency_dt;
          //steer_value is negative
          double epsi1 = epsi0 - v0 * atan(coeffs[1]) / Lf * latency_dt;
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
