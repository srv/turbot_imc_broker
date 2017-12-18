# turbot_imc_broker


This package communicates the Turbot/Sparus architecture with Neptus through the ros_imc_broker package that transform ROS msgs to IMC msgs.

## How to execute it
A part of this package you need to install and configure several things before running it.

### Install
* Install ros-imc-broker: git@github.com:srv/ros-imc-broker.git
* Install cyclops_rhodamine_ros: git@github.com:srv/cyclops_rhodamine_ros.git
* Install ros_arduino_bridge: https://github.com/hbrobotics/ros_arduino_bridge.git

### Configure
* Edit files in launch and vehicle-defs according to your configuration.
* Copy files in vehicle-defs folder to NEPTUS_HOME/vehicle_defs/

### To run
* Start the AUV architecture (real or simulated).
* Execute turbot or sparus launch (~/launch)
* Start Neptus. Open a console based on lauv.ncon and select turbot or sparus through menu: Advanced/Set main vehicle.

## Issues:
* Neptus can connect with ROS but messages from Neptus do not arrive to ROS.
 