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

#### Neptus:
* From the Neptus *Desktop* opne `IMC Monitor -> All Messages -> Estimated state or PlanControlState`. 
This window shows vehicle or mission related status.
* Open new console: `Consoles -> Open -> <NEPTUS_DIR>/conf/consoles/lauv.ncon`. 
A voice should anounce that your vehicle is connected.

To program a Mission plan:
* From the console push `Plan Edition` and select your vehicle name.
* To ass points in the mission: Press mouse right button and select `Add goto`.
* Select *goto* information (depth, speed, ...) from the panel that appears when clicking a goto waypoint.
* To send the mission to your vehicle: select your vehicle name in the top right corner of the console and press the green walking man.

## Issues:
* Neptus can connect with ROS but messages from Neptus do not arrive to ROS.
 