This page is a simple resume of the documentation of the [official documentation](http://turtlebot3.robotis.com/en/latest/introduction.html)

# Basic Commands

### Bringup
PC 
```sh
roscore
```

TB  
```sh
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Tele-op

PC  
```sh
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### SLAM

PC  
```sh
roslaunch turtlebot3_slam turtlebot3_slam.launch
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz
```

### Navigation
PC  
```sh
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz
```

### Simulation
PC  
```sh
roslaunch turtlebot3_fake turtlebot3_fake.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
OR
```sh
roslaunch turtlebot3_fake turtlebot3_fake.launch
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
