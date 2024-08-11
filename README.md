# Crobot

This workspace contains packages for the autonomous robot of VTCRO southeast con team.

## Getting Started

### Setup Environment

This workspace is designed to work with ROS Humble on Ubuntu.
You may also run the provided ROS Simulation Environment using Docker.
Clone this repository into your desired working directory.

### Install Dependencies
Navigate to the workspace folder and run the following commands
```
$ sudo apt update
$ source install/setup.bash
$ rosdep update
$ rosdep --install-from-paths src -y --ignore-src
```

### How to Run
To run a simulated test environment launch `bringup_simulated.launch.py` from the the `crobot_bringup` package.
To run the stack with your real hardware, launch `bringup.launch.py` from the `crobot_bringup` package.

See the individual package README's for additional information on configuration.