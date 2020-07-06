# Rover Config

## Overview

This package contains config files for launch files and rviz, gazebo  and robot models (URDF, XACRO).

**Keywords:** rover, config, urdf, xacro, gazebo, rviz

### License

The source code is released under a [TODO: Add License]().

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The Rover Config package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation


### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/rover_config.git
	cd ../
	colcon build

## config

* **marta.yaml** Contains parameters specific to MaRTA to be used in launch files.

* **nav2_params.yaml** Contains parameters specific to the navigation2 stack.

## maps

* **turtlebot3_world(\*.pgm & \*.yaml)** Contains map used in nav2

* **tb3_world_big(\*.pgm & \*.yaml)** Contains upscaled map used in nav2 so MaRTA actually fits

## models

* **turtlebot3_world/** Turtlebot world

## rviz
Contains .rviz files that can be loaded to visualize different rovers in Rviz

* **gamepad_sim.rviz** To be used with the gamepad simulation.

* **nav2_default_view.rviz** To be loaded with the nav2 stack.

* **rover_gazebo.rviz** Visualizes the rover fully in rviz. Including its position in space.

* **simple_sim.rviz** To be used if no position information is provided. Rover is positioned at (0, 0, 0)



## urdf

The following two file types can be found in *'urdf/'*:

* *'\*.gazebo'* contains gazebo specific plugins and parameters.

* *'\*.xacro'* conatains a urdf model for usage in ros and gazebo.


The following files can be found in *'urdf/'*:

* **exoter.\*** Model description of ExoTeR.

* **marta.\*** Model description of MaRTA.

* **ptu.\*** Model description of the small PTU found on ExoTeR and MaRTA.

* **bb2.\*** Model description of Bumblebee2 camera. Used for navigation and localization camera on MaRTA, ExoTeR and HDPR.

* **vlp_16.\*** VLP-16 (Puck) Velodyne LiDAR sensor.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz