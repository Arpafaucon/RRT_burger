# RRT-Star project for Turtlebot & ROS

NOTE : code documentation can be build with `make doc`

- [General Structure](#general-structure)
- [Standalone](#standalone)
    - [How to run it](#how-to-run-it)
        - [Input](#input)
        - [Ouput](#ouput)
    - [Displaying the results](#displaying-the-results)
        - [Prequisites](#prequisites)
        - [Usage](#usage)
        - [Settings](#settings)
- [ROS Plugin](#ros-plugin)
    - [Internals](#internals)
    - [Parameters](#parameters)
    - [Testing :  rrt_planner_tester node](#testing-rrtplannertester-node)
        - [Usage](#usage)
        - [Parameters](#parameters)
    - [Visualization](#visualization)

## General Structure

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

## Standalone

This executable demonstrates the use of the librairy without ROS. It was developped first to evaluate the potential of the RRTS algorithms

### How to run it

The program reads an input `.exp` file and writes as output a `.sol` file.
It can be launched like

```shell
./rrtstar file.exp
```

#### Input

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

#### Ouput

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

### Displaying the results

There is a basic python tool to display the results.

#### Prequisites

The tool obviously needs python installed, but any version after 2.7 should be fine.
Needed modules : 
- Matplotlib
- Numpy

#### Usage

From the root folder (..../rrt_planner/), launch the viz tool by 
```shell
python viz/viz.py exp/expTitle.sol
```

#### Settings

In `viz/viz.py`, it is possible to enable/disable the display of the RRT tree.


## ROS Plugin

Th main objective of the project was **to develop a motion planning plugin for ROS** using a RRTS (then BoxRRT) algorithm. 

### Internals

The plugin implements the `NavCore::BaseGlobalPlanner` plugin interface.

### Parameters

The planner reads the following parameters out of the `ROS Parameter Server`. They are expected in the **private plugin namespace**: 



 name              | type      | Description                                       |Default| Unit 
 -------------     |:------    | :------------                                     |-----: | :--- 
`verbose`          | bool      | enable plugin verbose output                      | true  | .
`publish_markers`  | bool      | enable pusblishing visual markers for Rviz        | true  | .
`max_iteration`    | int       | max iteration number                              | 5000  | .
`rrts_gamma`       | double    | gamma parameter of the planner                    | 0.5   | .
`goal_size`        | double    | tolerancy around given goal state                 | 0.1   | m
`goal_bias`        | double    | (S) goal bias factor in RRTS                          | 0.1   | m
`waypoint_distance`| double    | (S) max distance between two states                   | 0.1   | m
`robot_radius`     | double    | (S) Robot radius                                      | 0.05  | m
`discretization_step`|double   | (S) Integration step (affects collion test precision) | 0.01  | m

Parameters tagged with (s) are directly transmitted to `system_ros`. Only a distance conversion operation is performed.

`test.launch` is a good example of how to set these parameters.

### Testing :  rrt_planner_tester node

The `test_ros` files define a ROS node that creates and fills a ROS `costmap_2d::Costmap2D` object, using an `.exp` configuration files (mentionned above). It then calls the global planner interface, and writes the resulting plan in a file following the `.sol` conventions.

**Those output files are compatible with the `viz.py` visualization tool.**

#### Usage

This a standard ros node that can be launched with rosrun.

However the Costmap2DRos wrapper requires some tf2 transformation to be defined, the best is therefore to use the provided `test.launch` that runs a minimal working example.

```
roslaunch ./test.launch
```

Neeed **tf** transformations : 
- world <-> map
- map <-> base_link

#### Parameters

The node accepts the following parameters : 

- `f` : (string) The experience file to be runned on
- `odir` : (string) The folder where to write the output `.sol` files
- `res` : (double) The costmap resolution (size of a cell, in meters)



### Visualization 

In addition to returning the path to the caller, the plugin publish rviz `Markers` to present its results. Those can be found on the follwing topics : 
- `rrts_ros_path` gives additionnal visual informations on the path found. The boxes define a *safe zone* around each waypoint.
- `rrts_ros_tree` gives additionnal visual informations on the RRT tree. It should display the final tree, but some unknown issues still have to be solved. 