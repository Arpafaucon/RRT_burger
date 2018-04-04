# Simulation with Gazebo

- [Prerequisites](#prerequisites)
- [Usage](#usage)
- [Parameters](#parameters)

It is possible to setup a virtual experiment with gazebo to evaluate the planner in almost-real conditions. The computer simulates a fake robot (its **tf** tree, its laser scans, etc.), with which the planner plugin can be tested in its actual calling conditions (as a plugin, ie. a dynamically-bound library called by the `move_base` node).

## Prerequisites

The simulation needs a complete PC setup as described in the *Turtlebot3* installation manual. 
You must therefore check that the following package stacks are installed and built: 
- turtlebot3
- turtlebot3_msgs
- turtlebot3_simulationst

The `ROS_HOSTNAME` and `ROS_MASTER_URI` must be properly set (pointing to localhost). 
The `TURTLEBOT3_MODEL` must also be set.

See following settings :
```sh
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export TURTLEBOT3_MODEL=burger
```

## Usage

The needed `launch` files are in the `simulation` folder. 

```sh
# Gazebo simulation engine + visual client
roslaunch simulation/s1-gazebo.launch
# pre-configured Rviz visualisation 
roslaunch simulation/s2-rviz.launch
# pre-configured move_base with the right planners
roslaunch simulation/s3-navigation.launch
```

The launch rder isn't crucial, but seems to reduce crash/error chances.

## Parameters

Plugin parameters are specified in `simulation/plannerParams.yaml`. Namespaces are as follows : 
Parameter name      | plugin
:---                | :---
RRTPLanner/*        | Global Planner
BurgerPlannerROS/*  | Local Planner