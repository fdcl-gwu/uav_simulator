|![TravisCI](https://img.shields.io/badge/travis%20ci-%232B2F33.svg?style=for-the-badge&logo=travis&logoColor=white) | ![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)|
|-|-|
|[![Build Status](https://app.travis-ci.com/fdcl-gwu/uav_simulator.svg?branch=main)](https://app.travis-ci.com/fdcl-gwu/uav_simulator) | [tag:ros2-iron](https://hub.docker.com/layers/kanishgama/uav_simulator/ros2-iron/images/sha256-a18af779d63b68b9d6c3b7c522274f629c28e5de9da088285ce2ea7f12ee2d3c?context=repo) |

# Python - Gazebo Simulation Environment for a UAV with Geometric Control

This repository includes Python codes for the position control a UAV in a Gazebo simulation environment, using [geometric controllers](https://github.com/fdcl-gwu/uav_geometric_control).

## Features
* Developed using Python
* Uses a geometric controller that works great with aggressive maneuvers
* Uses Gazebo as the physics engine
* Has a nice GUI for controlling the UAV
* Estimator, controller, and trajectory generators are in their own ROS nodes. If you need to test your own estimator, controller, or a trajectory, you only need to modify the respective node.

![Landing](images/trajectory.gif)

### Why Python?
* Python makes developing/debugging easier and shorter (no compiling)
* Can easily find open-source modules or libraries for different tasks

## Which controller is used for the UAV control?
* A geometric controller with decoupled-yaw attitude control is used
* The controller is published in:
    ```sh
    @InProceedings{Gamagedara2019b,
        title={Geometric controls of a quadrotor uav with decoupled yaw control},
        author={Gamagedara, Kanishke and Bisheban, Mahdis and Kaufman, Evan and Lee, Taeyoung},
        booktitle={2019 American Control Conference (ACC)},
        pages={3285--3290},
        year={2019},
        organization={IEEE}
    }
    ```
* Implementation of the same controller in C++ and Matlab can be found at [https://github.com/fdcl-gwu/uav_geometric_control](https://github.com/fdcl-gwu/uav_geometric_control)



## Which estimator is used for the state estimation?
* The estimator defined in the following paper is implemented here (except for the sensor bias estimation terms):
    ```sh
    @InProceedings{Gamagedara2019a,
        author    = {Kanishke Gamagedara and Taeyoung Lee and Murray R. Snyder},
        title     = {Real-time Kinematics {GPS} Based Telemetry System for Airborne Measurements of Ship Air Wake},
        booktitle = {{AIAA} Scitech 2019 Forum},
        year      = {2019},
        month     = {jan},
        publisher = {American Institute of Aeronautics and Astronautics},
        doi       = {10.2514/6.2019-2377}
    }
    ```
* Matlab implementation of the above controller can be found at [https://github.com/fdcl-gwu/dkf-comparison](https://github.com/fdcl-gwu/dkf-comparison).
* Note that the Matlab implementation has a delayed Kalman filter that has not been implemented here. Only the non-delayed parts inside `DelayedKalmanFilter.m` is utilized here.



## Setting-up

### Releases

The current `main` branch has been tested to work on Ubuntu 22.04, running ROS2-Iron. Check [releases](https://github.com/fdcl-gwu/uav_simulator/releases) for different OS/ROS versions.
1. [v1.0](https://github.com/fdcl-gwu/uav_simulator/tree/v1.0): Ubuntu 18.04 with ROS-Melodic
1. [v2.0](https://github.com/fdcl-gwu/uav_simulator/tree/v2.0): Ubuntu 20.04 with ROS-Noetic
1. [v3.0](https://github.com/fdcl-gwu/uav_simulator/tree/v3.0): Ubuntu 22.04 with ROS2-Iron

:bangbang: If you are trying to use an older release, please checkout that release, and use setup instructions there.
Each release has different instructions.

:bangbang: If you are running this on a virtual machine, please make sure that Gazebo can run at real-time speed.
It is known that this simulation exhibits unintended behavior if the "real-time factor" of the Gazebo simulation is not closer to 1.0 (See [issue #3](https://github.com/fdcl-gwu/uav_simulator/issues/3)).

### Setting-up the repository
1. Clone the repository.
    ```sh
    git clone https://github.com/fdcl-gwu/uav_simulator.git
    ```
1. Update the submodules.
    ```sh
    cd uav_simulator
    git submodule update --init --recursive
    ```

### Dependencies

You have to options here:
1. Installing everything locally
1. Running a docker container

Installing everything locally is probably the most straight-forward way, but you have to instal dependencies manually, or may have to deal with package version changes. 
Docker solves this by streamlining all the dependencies, up-to the OS.
For example, if you are on Ubuntu 22.04 and want to test the ROS-Melodic version, docker will be the only way.

If you want to install everything locally, follow [Local Install](#local-install).
If you want to run a docker container instead, skip to [Docker Setup](#docker-setup).

#### Local Install
1. [ROS2](http://wiki.ros.org/): this repository has been developed using ROS2 Iron, on Ubuntu 22.04. If you are on a different version of Ubunto or ROS, please check the previous releases before installing dependencies. We recommend installing the ROS2 full version.

1. Python modules: these libraries must be installed in the system
    1. NumPy
    1. Pandas
    1. Matplotlib
    ```sh
    python3 -m pip install numpy pandas matplotlib
    ```

Now, skip to [Setting-up the plugins and Gazebo](#setting-up-the-plugins-and-gazebo).

#### Docker Setup

The instructions here assume you are on Ubuntu.
This has not been tested on other OS versions.

1. Install docker following [official instructions](https://docs.docker.com/engine/install/ubuntu/).
1. If you are not already there, `cd uav_simulator`
1. Enable xhost (required for Gazebo and GUI): `xhost +`
1. Build the docker image: `docker build -t uav_simulator .` (see following paragraph if you just want to pull the already built image instead)
1. Run a container: `bash docker_run.sh`

The last command will start a docker container, install all the dependencies, and mount the local directory there.
The first time you run the build command will take a while as it installs all the libraries.

You can skip the build command altogether by pulling the built docker from the Docker Hub with the following command.
This is NOT required if you are building it locally using the build command.
```sh
docker pull kanishgama/uav_simulator:ros2-iron
bash docker_run.sh
```

After that, you only need to run the `bash docker_run.sh` every time you need to run the simulation.
Since this mounts the local repository inside the docker, you just need to change the code in your local repository, and it will be automatically update inside the docker.

For running the code, simply follow [Setting-up the plugins and Gazebo](#setting-up-the-plugins-and-gazebo), and onwards.


### Setting-up the plugins and Gazebo
You only need to do the followings once (unless you change the Gazebo plugins)
1. Make the plugging.
    ```sh
    # From uav_simulator
    colcon build
    ```
1. Source the relevant directories (**NOTE**: you need to do this on every new terminal).
    ```sh
    # From uav_simulator
    source install/local_setup.bash
    ```

### Running the simulation environment 
1. In the current terminal window, launch the Gazebo environment:
    ```sh
    # From uav_simulator
    ros2 launch uav_gazebo uav_gazebo.launch.py 
    ```
1. Once the Gazebo is launched, run the UAV code from a different terminal (if you already don't know, you may find [**tmux**](https://github.com/tmux/tmux/wiki) a life-saver):
    ```sh
    # From uav_simulator
    ros2 launch fdcl_uav fdcl_uav_launch.py
    ```

    Everytime you change the Python code, run the following commands
    ```sh
    # From uav_simulator
    colcon build --packages-select fdcl_uav
    ros2 launch fdcl_uav fdcl_uav_launch.py
    ```
    The code has been tested with Python3.10.12, which comes default with Ubuntu 22.04.


![Terminal](images/running.gif)

### Tips
1. Every time you change the simulation environment, you have to kill the program, `colcon build` and re-run it. 
1. If you do not make any changes to the simulation environment, you only need to kill the Python program. 
<!-- 1. The UAV will re-spawn at the position and orientation defined in `reset_uav()` in `rover.py` when you run the Python code. -->

## Control Guide
* Simply click on the buttons on the GUI to control the UAV.
* You can easily switch between each trajectory mode simply clicking on the radio buttons.
* Stay mode simply commands the UAV to stay at the current position.
* When take-off, stay, and circle trajectories end, the UAV switches to the "manual" mode.
* When the UAV is in manual, you can use following keys (these are not case sensitive):
  * `WASD`: to move in horizontal plane
  * `P`: increase altitude
  * `L`: decrease altitude
  * `Q`: yaw rotation in anti-clockwise direction
  * `E`: yaw rotation in clockwise direction
* At any point of the flight, you can use following keys (these are not case sensitive):
  * `M`: kill motors
  * `0-5`: set the flight mode without clicking on the GUI
* Please not that the GUI must be in focus for any of the above keys to work.
* If you want to change the above keyboard shortcuts, you can do so by editing `on_key_press` function in `gui.py`.

## Running Unit-Tests
* Make sure you are in the main directory.
* Run `python -m unittest`.
* Unit tests have only been tested on Python 3.9.
* Currently, unit test only covers the `matrix_utils.py` module. 

<!-- `export GAZEBO_PLUGIN_PATH=/home/kani/Documents/uav_simulator/build:$GAZEBO_PLUGIN_PATH` -->