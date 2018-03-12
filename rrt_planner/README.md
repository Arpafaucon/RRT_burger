# RRT-Star project for Turtlebot & ROS

## Projects

### General Structure

The structure of programs using the RRT capabilities can be separated in 3 categories

1. **The invariable source files** : those files shouldn't be modified at all.
    - `kdtree.c`
    - `rrts.h` & `rrts.hpp`
    - `system.h`


2. **The system description**: those files implements the interfaces defineD in the file `system.h` listed above. They are therefore context-dependent.
    - `system_ros.h` & `system_ros.cpp`
    - `system_single_integrator.h` & `system_single_integrator.cpp`

3. **User code**:
    - `rrts_standalone.cpp` is a basic program
    - `rrts_ros.h` & `rrts_ros.cpp` define a ROS `NavCore::BaseGlobalPlanner` plugin
    - `rest_ros.cpp` is a ROS node testing the plugin.

### Standalone

This executable demonstrates the use of the librairy without ROS. It was developped first to evaluate the potential of the RRTS algorithms

#### How to run it

The program reads an input `.exp` file and writes as output a `.sol` file.
It can be launched like

```shell
./rrtstar file.exp
```

##### Input

The standalone takes its instructions from `exp` files. These follows a precise template, defined in the templates files of the exp folder, and copied below

```
# TEMPLATE for experience
space-separated values, one topic per line
-- begin template --
title
D x y w h 	// position of the space width & height
S x y 		// start point coords x&y
G x y w h 	// goal region (x,y) (lower left point) width & height
O x y w h	// obstacle x,y,w,h (same rule) //MULTIPLE
-- end template --

-- begin example --
basicExp
D 20 20
S 0 0
G 10 10 1 1
O 4 4 1 1
O 15 12 2 1
-- end example --
```

##### Ouput

The output file is named following the title given in the experience file. The template of the resulting solution file is as follows :

```
###
# Solution file format
###
# space-separated values, one topic per line
#
-- begin template --
title 					// string		: title of this solution
experienceFileName 		// string		: related file describing the experience
G x y					// char float*2	: goal position actually reached
W x1 y1					// char float*4	: path segment to reach the goal	//MULTIPLE
...
T x1 y1 x2 y2			//char float*4	: segment of the RRT tree			//MULTIPLE
...
-- end template --
```

#### Displaying the results

There is a basic python tool to display the results.

##### Prequisites

The tool obviously needs python installed, but any version after 2.7 should be fine.
Needed modules : 
- Matplotlib
- Numpy

##### Usage

From the root folder (..../rrt_planner/), launch the viz tool by 
```shell
python viz/viz.py exp/expTitle.sol
```

##### Settings

In `viz/viz.py`, it is possible to enable/disable the display of the RRT tree.


### ROS Plugin

Th main objective of the project was **to develop a motion planning plugin for ROS** using a RRTS (then BoxRRT) algorithm. 

The plugin implements the `NavCore::BaseGlobalPlanner` plugin interface.

#### Simulation & Testing

##### ROS rrt_planner_tester node

The `test_ros` files define a ROS node that creates and fills a ROS `costmap_2d::Costmap2D` object, using an `.exp` configuration files (mentionned above). It then calls the global planner interface, and writes the resulting plan in a file following the `.sol` conventions.

**Those output files are compatible with the `viz.py` visualization tool.**
