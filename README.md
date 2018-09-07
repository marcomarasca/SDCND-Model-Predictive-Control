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
[epsi_w_epsi_20]: ./images/w_epsi_100.png "EPSI graph using a higher weight for epsi: 100"
[steering_w_delta_low]: ./images/w_delta_10.png "Steering graph using a low weight for input delta: 10"
[steering_w_delta_high]: ./images/w_delta_5000.png "Steering graph using a higher weight for input delta: 5000"
[steering_w_delta_d_low]: ./images/w_delta_d_10.png "Steering graph using a low weight for the steering jerkiness: 10"
[steering_w_delta_d_high]: ./images/w_delta_d_2500.png "Steering graph using a medium weight for the steering jerkiness: 2500"
[steering_w_delta_d_high]: ./images/w_delta_d_10000.png "Steering graph using a high weight for the steering jerkiness: 10000"

[cte_dt_80]: ./images/w_dt_80.png "CTE graph using a lower step delta t"
[cte_dt_115]: ./images/w_dt_115.png "CTE graph using a higher step delta t"
[cte_n_8]: ./images/w_n_8.png "CTE graph using a lower number of steps"
[cte_n_10]: ./images/w_n_10.png "CTE graph using a medium number of steps"
[cte_n_12]: ./images/w_n_12.png "CTE graph using a higher number of steps"

![alt text][sim_gif]

Overview
---

This repository contains a C++ implementation of a model predictive control (MPC) controller that is used in order to direct a vehicle to follow a desired trajectory.

An MPC is an alternative and more advanced method of process control (e.g. See the [PID Control](https://github.com/Az4z3l/CarND-PID-Controller) project for a simple version of process control) that is used to control a process while satisfying a set of constraints. In particular the problem is reframed in a way to reduce it to an optimization problem. In our case the solution to the optimization problem is the "ideal" trajectory that the vehicle should follow.

The input trajectory may be provided as a set of way-points (or coefficients to describe a line) and the MPC constructs a model of the physics of the vehicle trying to predict how the vehicle will behaves at the various timesteps along the trajectory. The optimization occurs while matching the state of the vehicle according to its model to the ideal trajectory, minimizing a given cost function to provide the closest match between the vehicle "predicted" trajectory to the ideal input trajectory.

In other words the MPC knows the model of the vehicle and given an initial state simulates different "actuator inputs" (e.g. throttle and steering) that produce different trajectories and selecting the one that minimizes the cost.

TODO
- Describe [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) inputs and outputs
- Describe the model (state, actuators and updates)
- Describe the Cost function and constraints
- Describe the steps prediction and delta t
- Describe dealing with latency
- Describe parameter tuning

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


