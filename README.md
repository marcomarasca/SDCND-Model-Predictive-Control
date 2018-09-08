# Control: MPC Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[sim_gif]: ./images/run_70.gif "Simulator using the MPC controller"
[sim_video]: ./images/run_70.mp4 "Simulator using the MPC controller"
[throttle_w_a_low]: ./images/w_a_0.2.png "Throttle graph using a low weight for input a: 0.2"
[throttle_w_a_high]: ./images/w_a_1.2.png "Throttle graph using a higher weight for input a: 1.2"
[throttle_w_delta_a_low]: ./images/w_delta_a_0.5.png "Throttle graph using a low weight for the throttle jerkiness: 0.5"
[throttle_w_delta_a_high]: ./images/w_delta_a_2500.png "Throttle graph using a higher weight for the throttle jerkiness: 2500"
[cte_w_cte_low]: ./images/w_cte_5.png "CTE graph using a low weight for cte: 5"
[cte_w_cte_high]: ./images/w_cte_20.png "CTE graph using a higher weight for cte: 20"
[epsi_w_epsi_20]: ./images/w_epsi_20.png "EPSI graph using a low weight for epsi: 20"
[epsi_w_epsi_100]: ./images/w_epsi_100.png "EPSI graph using a higher weight for epsi: 100"
[steering_w_delta_low]: ./images/w_delta_10.png "Steering graph using a low weight for input delta: 10"
[steering_w_delta_high]: ./images/w_delta_5000.png "Steering graph using a higher weight for input delta: 5000"
[steering_w_delta_d_low]: ./images/w_delta_d_10.png "Steering graph using a low weight for the steering jerkiness: 10"
[steering_w_delta_d_medium]: ./images/w_delta_d_2500.png "Steering graph using a medium weight for the steering jerkiness: 2500"
[steering_w_delta_d_high]: ./images/w_delta_d_10000.png "Steering graph using a high weight for the steering jerkiness: 10000"

[cte_dt_80]: ./images/w_dt_80.png "CTE graph using a lower step delta t"
[cte_dt_115]: ./images/w_dt_115.png "CTE graph using a higher step delta t"
[cte_n_8]: ./images/w_n_8.png "CTE graph using a lower number of steps"
[cte_n_10]: ./images/w_n_10.png "CTE graph using a medium number of steps"
[cte_n_12]: ./images/w_n_12.png "CTE graph using a higher number of steps"

![alt text][sim_gif]

Overview
---

This repository contains a C++ implementation of a [model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control) (MPC) that is used in order to direct a vehicle to follow a desired trajectory.

An MPC is an alternative and more advanced method of process control (e.g. See the [PID Control](https://github.com/Az4z3l/CarND-PID-Controller) project for a simple version of process control) that is used to control a process while satisfying a set of constraints. In particular the problem is reframed in a way to reduce it to an optimization problem. In our case the solution to the optimization problem is the "ideal" trajectory that the vehicle should follow.

The input trajectory may be provided as a set of way-points (or coefficients to describe a line) and the MPC constructs a model of the physics of the vehicle trying to predict how the vehicle will behaves at the various timesteps along the trajectory. The optimization occurs while matching the state of the vehicle according to its model to the ideal trajectory, minimizing a given cost function to provide the closest match between the vehicle "predicted" trajectory to the ideal input trajectory.

In other words the MPC knows the model of the vehicle and given an initial state simulates different **actuator inputs** (e.g. throttle and steering) that produce different trajectories and selecting the one that minimizes the cost.

In this project the desired trajectory is fed by a simulated environment thanks to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) through [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file processes the incoming messages, parsing the data that is then processed by the [MPC](./src/MPC.cpp) class.

The input to the program (e.g. each websocket message) contains the following attributes:

* **ptsx** A list of x positions in map's coordinates of the trajectory
* **ptsy** A list of y positions in map's coordinates of the trajectory
* **x** The current x position of the vehicle in map's coordinates
* **y** The current y position of the vehicle in map's coordinates
* **psi** The vehicle orientation in radians, in respect to the global map
* **steering_angle** The steering angle in radians
* **throttle** The current throttle value (in the range [1, -1])
* **speed** The current speed in mph

The program outputs (e.g. as a reply in the websocket message as expected by the simulator) the following:

* **steering_angle** The steering angle value to submit next (in the range [1, -1], NOT radians)
* **throttle** The throttle value to submit next (in the range [1, -1])
* **next_x** A list of x positions for the reference trajectory (in the vehicle's coordinates system)
* **next_y** A list of y positions for the reference trajectory (in the vehicle's coordinates system)
* **mpc_x** The list of x positions computed by the MPC for the minimized cost trajectory
* **mpc_y** The list of y positions computed by the MPC for the minimized cost trajectory

With the information in input (state) the MPC predicts the trajectory for a given number of steps and thanks to the model and
the constraints on it uses a solver to optimize the trajectory that reduces the cost, outputting the ideal actuations values
to reach that trajectory. Note that while the solver computes the actuations at each step, what we need and send back to the simulator is only the first step actuations, then at the next message we repeat the process (e.g. recomputing the trajectory and actuations according to the new state vector).

To ease with tuning and testing the program takes in input an optional *[configuration file](./config.json)* which is a simple text file containing a json object with various parameters (described in more details in the next sections):

```
{
    "delay": 100,         // The simulated delay for the actuations
    "steps_n": 10,        // Number of steps for the MPC optimizer
    "step_dt": 115,       // Time in milliseconds between each step
    "speed": 70,          // Target speed
    "weights": {          // Weights for the cost function
        "cte": 5,         // Weight applied for the cross-track error
        "epsi": 100,      // Weight applied for the orientation error
        "v": 0.5,         // Weight applied for the delta between current speed and the target
        "delta": 5000,    // Weight applied to the steering value
        "a": 1.2,         // Weight applied for the throttle value
        "delta_d": 5000,  // Weight applied to the delta between consecutive steering values
        "a_d": 1500       // Weight applied to the delta between consecutive throttle values
    }
}
```

### Model

For this project we use a the so called [kinematic model](https://en.wikipedia.org/wiki/Kinematics) in order to simplify the physics of the vehicle reducing it to a *bicycle* model, where we consider the tires steering at the same angle on a single axes, this model is simplistic as it does not take into account the multitude of forces that are applied in the real world but it is proven to be effective at lower speeds and close enough to make accurate predictions of the vehicle behavior.

The *state vector* <a href="https://www.codecogs.com/eqnedit.php?latex=[x,&space;y,&space;\psi,&space;v]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?[x,&space;y,&space;\psi,&space;v]" title="[x, y, \psi, v]" /></a> includes the following data:

* **x** Position x of the vehicle
* **y** Position y of the vehicle
* **psi** Heading of the vehicle
* **v** Velocity of the vehicle

The considered *actuations* are simply the **steering angle** (delta) the **throttle** (a), which are the values that the MPC will optimize.

With this in mind we can simply apply the kinematic model to predict the state at time t+1 from the previous state at time t; Considering the vector <a href="https://www.codecogs.com/eqnedit.php?latex=[x,&space;y,&space;\psi,&space;v]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?[x,&space;y,&space;\psi,&space;v]" title="[x, y, \psi, v]" /></a> as the vehicle state and <a href="https://www.codecogs.com/eqnedit.php?latex=[\delta,&space;a]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?[\delta,&space;a]" title="[\delta, a]" /></a> as the actuations, the following equations can be used:

<a href="https://www.codecogs.com/eqnedit.php?latex=\newline&space;x_{t&plus;1}&space;=&space;x_{t}&space;&plus;&space;v&space;*&space;\cos(\psi)&space;*&space;dt&space;\newline&space;y_{t&plus;1}&space;=&space;y_{t}&space;&plus;&space;v&space;*&space;\cos(\psi)&space;*&space;dt&space;\newline&space;\psi_{t&plus;1}&space;=&space;\psi_{t}&space;&plus;&space;\delta_{t}\tfrac{v_{t}}{L_{f}}&space;*&space;dt&space;\newline&space;v_{t&plus;1}&space;=&space;v_{t}&space;&plus;&space;a_{t}&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\newline&space;x_{t&plus;1}&space;=&space;x_{t}&space;&plus;&space;v&space;*&space;\cos(\psi)&space;*&space;dt&space;\newline&space;y_{t&plus;1}&space;=&space;y_{t}&space;&plus;&space;v&space;*&space;\cos(\psi)&space;*&space;dt&space;\newline&space;\psi_{t&plus;1}&space;=&space;\psi_{t}&space;&plus;&space;\delta_{t}\tfrac{v_{t}}{L_{f}}&space;*&space;dt&space;\newline&space;v_{t&plus;1}&space;=&space;v_{t}&space;&plus;&space;a_{t}&space;*&space;dt" title="\newline x_{t+1} = x_{t} + v * \cos(\psi) * dt \newline y_{t+1} = y_{t} + v * \cos(\psi) * dt \newline \psi_{t+1} = \psi_{t} + \delta_{t}\tfrac{v_{t}}{L_{f}} * dt \newline v_{t+1} = v_{t} + a_{t} * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\L_{f}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\L_{f}" title="\L_{f}" /></a> measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower the turn rate.

### Input trajectory

The simulator gives us a set of way-points as the desired trajectory. These points can fitted to a polynomial in order to simulate the trajectory, a *[3rd degree polynomial](./src/main.cpp#L64)* works quite well to simulate a curvy road.

Moreover, since the coordinates are given in the map's coordinate system it is useful to [convert](./src/main.cpp#L55) them into to vehicle coordinate system simply applying a transformation according to the vehicle position and orientation.

This transformation makes it simpler to compute the errors in the next sections, as the coordinate system is now relative to the car's point of view, meaning that the initial state vector can consider the position and orientation as 0 (e.g. nose of the car).

### Errors and Cost Function

Additionally to the vehicle state we can use the input trajectory to compute a measure of error of the vehicle position and orientation, in particular:

* **Cross Track Error (CTE)** We consider the distance between the current position and the desired position (given from the trajectory in input)
* **Orientation Error (EPSI)** Simply the difference in the current orientation and the desired orientation (given from the trajectory in input)

Or in other terms:

<a href="https://www.codecogs.com/eqnedit.php?latex=\newline&space;cte_{t}&space;=&space;y_{t}&space;-&space;y_{desired}&space;\newline&space;e\psi_{t}&space;=&space;\psi_{t}&space;-&space;\psi_{desired}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\newline&space;cte_{t}&space;=&space;y_{t}&space;-&space;y_{desired}&space;\newline&space;e\psi_{t}&space;=&space;\psi_{t}&space;-&space;\psi_{desired}" title="\newline cte_{t} = y_{t} - y_{desired} \newline e\psi_{t} = \psi_{t} - \psi_{desired}" /></a>

We can compute the desired values of y and psi simply *fitting a polynomial* for the input trajectory points and computing the relative terms for x at time t (in the following using a 3rd degree polynomial):

<a href="https://www.codecogs.com/eqnedit.php?latex=\newline&space;y_{desired}&space;=&space;f(x_{t})&space;=&space;ax^{3}&space;&plus;&space;bx^{2}&space;&plus;&space;cx&space;&plus;&space;d&space;\newline&space;\psi_{desired}&space;=&space;\arctan({f}'(x_{t}))&space;=&space;\arctan(3ax^2&space;&plus;&space;2bx&space;&plus;&space;c)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\newline&space;y_{desired}&space;=&space;f(x_{t})&space;=&space;ax^{3}&space;&plus;&space;bx^{2}&space;&plus;&space;cx&space;&plus;&space;d&space;\newline&space;\psi_{desired}&space;=&space;\arctan({f}'(x_{t}))&space;=&space;\arctan(3ax^2&space;&plus;&space;2bx&space;&plus;&space;c)" title="\newline y_{desired} = f(x_{t}) = ax^{3} + bx^{2} + cx + d \newline \psi_{desired} = \arctan({f}'(x_{t})) = \arctan(3ax^2 + 2bx + c)" /></a>

Accordingly we can compute the CTE and EPSI at time t + 1 considering the current error plus the change in error due to the vehicle movement:

<a href="https://www.codecogs.com/eqnedit.php?latex=\newline&space;cte_{t&plus;1}&space;=&space;cte_{t}&space;&plus;&space;v_{t}&space;*&space;\sin(e\psi)&space;*&space;dt&space;=&space;y_{t}&space;-&space;f(x)&space;&plus;&space;v_{t}*&space;\sin(e\psi)&space;*&space;dt&space;\newline&space;e\psi_{t&plus;1}&space;=&space;e\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt&space;=&space;\psi_{t}&space;-&space;\arctan({f}'(x))&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\newline&space;cte_{t&plus;1}&space;=&space;cte_{t}&space;&plus;&space;v_{t}&space;*&space;\sin(e\psi)&space;*&space;dt&space;=&space;y_{t}&space;-&space;f(x)&space;&plus;&space;v_{t}*&space;\sin(e\psi)&space;*&space;dt&space;\newline&space;e\psi_{t&plus;1}&space;=&space;e\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt&space;=&space;\psi_{t}&space;-&space;\arctan({f}'(x))&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt" title="\newline cte_{t+1} = cte_{t} + v_{t} * \sin(e\psi) * dt = y_{t} - f(x) + v_{t}* \sin(e\psi) * dt \newline e\psi_{t+1} = e\psi_{t} + \frac{v_{t}}{L_{f}} * \delta_{t} * dt = \psi_{t} - \arctan({f}'(x)) + \frac{v_{t}}{L_{f}} * \delta_{t} * dt" /></a>

Now we can build our [cost function](./src/MPC.cpp#L34) that takes into account:

* The CTE and EPSI: To keep the vehicle near the desired trajectory
* The delta between the current speed and a target speed: To keep the vehicle running
* The actuation input: To reduce steering and accelerating/breaking too quickly
* The delta between actuation at time t and t + 1: to reduce jerkiness

<a href="https://www.codecogs.com/eqnedit.php?latex=\newline&space;\mbox{J&space;=&space;\sum_{t&space;=&space;1}^{N}(cte_{t}-cte_{ref})^2&plus;(e\psi_{t}-e\psi_{ref})^2&plus;(v_{t}-v_{ref})^2&plus;\delta_{t}^2&plus;a_{t}^2&plus;(\delta_{t&plus;1}-\delta_{t})^2&plus;(a_{t&plus;1}-a_{t})^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\newline&space;\mbox{J&space;=&space;\sum_{t&space;=&space;1}^{N}(cte_{t}-cte_{ref})^2&plus;(e\psi_{t}-e\psi_{ref})^2&plus;(v_{t}-v_{ref})^2&plus;\delta_{t}^2&plus;a_{t}^2&plus;(\delta_{t&plus;1}-\delta_{t})^2&plus;(a_{t&plus;1}-a_{t})^2}" title="\newline \mbox{J = \sum_{t = 1}^{N}(cte_{t}-cte_{ref})^2+(e\psi_{t}-e\psi_{ref})^2+(v_{t}-v_{ref})^2+\delta_{t}^2+a_{t}^2+(\delta_{t+1}-\delta_{t})^2+(a_{t+1}-a_{t})^2}" /></a>

Finally we can set some *constraints* on our model, such as [maximum steering range](./src/MPC.cpp#L213) and [maximum throttle range](./src/MPC.cpp#L219). With the model and the cost functions ready we can run our optimizer to find the solution that minimize the cost and outputs the desired actuations.

Note that in this implementation we added custom **weights** to each term of the cost function to empathize the effect of each term in the optimization. In particular:

##### CTE Weight

The weight on the cross-track error induces the vehicle to stay close to the desired trajectory, at the dispense of instability:

![alt text][cte_w_cte_high]

A lower value relaxes the vehicle behavior increasing the distance to the center:

![alt text][cte_w_cte_low]

##### EPSI Weight

The weight on the orientation error is helpful to force the car to follow a the orientation of the trajectory:

![alt text][epsi_w_epsi_100]

A value that is too low induces the vehicle to oscillate as it will try to steer more abruptly towards a lower CTE:

![alt text][epsi_w_epsi_20]

##### Actuations Weight

The weights on the actuations (steering delta and throttle a) is used to reduce the use of the actuations themselves (e.g. for a smoother driving). In the following we can see a comparison between using a lower (left) weight on the throttle actuation vs a higher (right):

![alt text][throttle_w_a_low]![alt text][throttle_w_a_high]

The higher the weight the smoother the acceleration of the car. A similar effect, but on the steering can be observed for the weight on the delta actuation:

![alt text][steering_w_delta_low]![alt text][steering_w_delta_high]

On the left, using a lower weight for delta the vehicle steers much more quickly.

##### Actuations Delta Weight

We can add a weight also on the delta between actuations at subsequent timesteps, this has the effect again to reduce the jittering when turning and accelerating/breaking:

![alt text][throttle_w_delta_a_low]![alt text][throttle_w_delta_a_high]

On the left a use of a lower weight for the difference between actuations of the throttle, on the right a higher weight inducing a smoother change.

Similarly for the steering delta (low on the left and high on the right):

![alt text][steering_w_delta_d_low]![alt text][steering_w_delta_d_high]

We can observe that a higher weight on the delta between steering steps reduces the jerkiness of the car when turning (since the steering is performed gradually in smaller steps).

### Number of Steps and Delta t

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. MPC uses two variables to determine until when the prediction is made, in particular we define the number of timesteps N and dt which is the amount of time between actuations.

The larger N the more variables the solver needs to take into account and therefore the larger the computation. The larger dt the less frequent the actuations are, making it less accurate in representing the continuos reference trajectory.

Ideally N * dt should be big enough to have a good future prediction, e.g. if N is 10 and dt is 0.1 then 1 second of trajectory is predicted for the future. Note that the higher the speed the longer the predicted trajectory will be (since the vehicle during the elapsed dt will have gained more distance). A prediction that goes too far into the future will be inaccurate since in the meanwhile the trajectory might have changed dramatically, on the other end a prediction that is too close is meaningless as there is not enough room to predict accurate actuators.

In this project I was aiming at having a target speed of around 70mph, and about 1 second of prediction in the future. In order to select the best value I plotted the effects on the CTE with different values of N and dt.

In the following we can see the reduced mean CTE of having a slightly higher dt (80ms on the left and 115 on the right), with a fixed N of 10:

![alt text][cte_dt_80]![alt text][cte_dt_115]

This is probably due to the fact that the simulated latency plus the average computation time is around 110ms and therefore the latter is a more realistic value.

I also tried out different values of N to see the effect, keeping a fixed value of dt of 115ms:

With **N = 8**:

![alt text][cte_n_8]

With **N = 10**:

![alt text][cte_n_10]

With **N = 12**:

![alt text][cte_n_12]

The difference is subtle, but I decided to stick to N = 10 as a lower value was not enough (higher CTE), while a higher value was giving bigger spikes around the turns (meaning that the prediction was a bit off for the quick changes due to the "high" speed).

### Latency

In this project we also take into account *latency* to predict correctly the desired trajectory. Since the time between the command is sent to the simulator and the actual values are transmitted to the simulated vehicle is extremely fast this is not a realistic scenario, therefore the program [adds a delay](./src/main.cpp#L162) of 100ms before transmitting the new actuations to simulate a real world environment where there might be some substantial delay before the actuations are actually performed.

To deal with this latency in this project, given that we know the simplified model of the vehicle, once we get the current state of the car we can simply [predict the next state](./src/main.cpp#L90) and use the new state vector to feed into the MPC.

Moreover notice that we also take into account the [computation delay](./src/main.cpp#L87) from the optimizer, the program records the average computation time and adds it to the simulated delay in order to have a more accurate prediction.

Getting Started
---

In order to run the program you need the simulator provided by [Udacity](https://www.udacity.com/) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install **[uWebSocketIO](https://github.com/uWebSockets/uWebSockets)** for either Linux or Mac systems. For windows you can use either Docker, VMware, or even better [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. The version compatible with the simulator is the uWebSocketIO branch **e94b6e1**.

The application uses **[Ipopt and CppAD](https://projects.coin-or.org/Ipopt)** to find a solution for the cost function, please refer to [this document](https://github.com/Az4z3l/CarND-MPC/blob/master/install_Ipopt_CppAD.md) for installation instructions.

Once uWebSocketIO, Ipopt and CppAD are installed, the main program can be built and run by doing the following from the project top directory.

1. ```mkdir build```
2. ```cd build```
3. ```cmake .. && make```
4. ```./mpc```

Note that to compile the program with debug symbols you can supply the appropriate flag to cmake: ```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```.

Now the Udacity simulator can be run selecting the MPC Control project, press start and see the application in action.

#### Other Dependencies

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
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.

Environment Setup
---

This project was developed under windows using the windows subsystem for linux ([WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)) with Ubuntu Bash 16.04 together with [Visual Studio Code](https://code.visualstudio.com/).

The steps to setup the environment under mac, linux or windows (WSL) are more or less the same:

- Review the above dependencies
- Clone the repo and run the appropriate script (./install-ubuntu.sh under WSL and linux and ./install-mac.sh under mac), this should install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) from the branch **e94b6e1**

Under windows (WSL) and linux you can make a clean installation as follows:

1. ```sudo apt-get update```
2. ```sudo apt-get install git```
3. ```sudo apt-get install cmake```
4. ```sudo apt-get install openssl```
5. ```sudo apt-get install libssl-dev```
6. ```git clone https://github.com/Az4z3l/CarND-MPC```
7. ```sudo rm /usr/lib/libuWS.so```
8. ```./install-ubuntu.sh```


Next install Ipopt:

- For mac:

  1. ```brew tap brewsci/science```
  2. ```brew install ipopt --with-openblas```

- For linux and windows (WSL):
  1. ```sudo apt-get install gfortran```
  2. ```sudo apt-get install unzip```
  3. ```wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip```
  4. ```sudo ./install_ipopt.sh Ipopt-3.12.7/```


#### Debugging with VS Code

Since I developed this project using WSL and Visual Studio Code it was very useful for me to setup a debugging pipeline. VS Code comes with a official Microsoft cpp extension that can be downloaded directly from the marketplace: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools. After the installation there are a few things to setup in order to make it work with the subsystem for linux, personally I went with the default Ubuntu distribution.

For the following setup I assume that the repository was cloned in **D:/Dev/CarND-MPC/**.

##### Setup the language server (for IntelliSense)

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md): 

Simply Crtl+P and select "C/Cpp: Edit Configurations", this will create a c_cpp_properties.json file that can be configured as follows:

```json
{
    "name": "WSL",
    "intelliSenseMode": "clang-x64",
    "compilerPath": "/usr/bin/gcc",
    "includePath": [
        "${workspaceFolder}"
    ],
    "defines": [],
    "browse": {
        "path": [
            "${workspaceFolder}"
        ],
        "limitSymbolsToIncludedHeaders": true,
        "databaseFilename": ""
    },
    "cStandard": "c11",
    "cppStandard": "c++17"
}
```

##### Setup the Debugger

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md):

First install gdb in the WSL:

```
sudo apt install gdb
```

Then simply create a lunch configuration from VS Code: "Debug" -> "Add Configuration.." and setup the launch.json as follows:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/mnt/d/Dev/CarND-MPC/build/mpc",
            "args": [],
            "stopAtEntry": false,
            "cwd": "/mnt/d/Dev/CarND-MPC/build/",
            "environment": [],
            "externalConsole": true,
            "windows": {
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
            },
            "pipeTransport": {
                "pipeCwd": "",
                "pipeProgram": "c:\\windows\\sysnative\\bash.exe",
                "pipeArgs": ["-c"],
                "debuggerPath": "/usr/bin/gdb"
            },
            "sourceFileMap": {
                "/mnt/d": "d:\\"
            }
        }
    ]
}
```

Note how the program is mapped directly into the file system of the WSL and piped through bash.exe (the paths are relative to the WSL environment).

Now you are ready to debug the application directly from VS Code, simply compile the application from within the WSL with the debug symbols:

```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```

And run the debugger from VS Code (e.g. F5) :)


