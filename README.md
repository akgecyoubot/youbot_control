# pybotserver
My KUKA youBot project. Server side.

This is a part of my thesis work.

For more info see project wiki.

## What's implemented?

- **Joypad control.** Execute `roslaunch pybotserver joypad_control.launch` in terminal to run.
- **Basic GUI control** with `scripts/client-gui.py`

## Installation

 1. *set up ROS Hydro and configure catkin workspace*
 2. `sudo apt-get install ros-hydro-youbot-driver-ros-interface`
 3. `git clone https://github.com/kirillmorozov/pybotserver.git ~/catkin_ws/src/`
 4. `cd ~/catkin_ws/`
 5. `catkin_make`
