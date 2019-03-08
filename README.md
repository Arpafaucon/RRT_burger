# burgerRRT

This package is intended to provide enhanced version of path-planning algorithms. It provides an implementation of ROS local and global planners plugin, allowing to develop refined path-planning algorithms while deeply understanding and controlling the computing process.

The package also comes with a set of visualization tools and technique, demonstrated with working examples, to help develop a new path-planner.

This is part of the P3A research project of **Amaury Camus & Grégoire Roussel**, under the supervision of **Adina M. Panchea**

## Burger Local Planner

author: Amaury Camus

This package implement the `NavCore::BaseLocalPLanner` interface.

## Burger Global planner

author: Grégoire Roussel & Amaury Camus

This pacakge implements the `NavCore::BaseGlobalPlanner`interface.
For the moment, it provides a adaptable RRTS engine, and numerous clients programs tailored for different needs :
- a standalone program
- a ROS plugin
- a standalone ROS node using the plugin.  

## Prerequisites

The planners need a complete PC setup as described in the *Turtlebot3* installation manual. 
You must therefore check that the following package stacks are installed and built: 
- turtlebot3
- turtlebot3_msgs
- turtlebot3_simulations

## Documentation

Both packages have a `README.md`.

Additionnaly, both packages have doxygen configured : run
```sh
doxygen doxy.conf
```
or simply
```sh
make doc
```
to generate a pretty html code API.

### How-to

http://docs.ros.org/api/catkin/html/howto/format2/building_libraries.html
