# Burger Local Planner

## Description

This is a ROS plugin implementing the NavCore::BaseLocalPlanner interface.
It is designed to work with a **Turtlebot3 burger robot**
The plugin provides a local planner library with the following assets:
- integrated dynamic model of the turtlebot
- stricter observance of global planner waypoints

## Parameters

The planner reads the following parameters out of the `ROS Parameter Server`. They are expected in the **private plugin namespace**: 

 name          | type  | Description                                               | Default| Unit 
 ------------- |:------| :------------                                             |-----:  | :--- 
`L`            | float | distance between middle of wheels                         | 0.16  | m
`acc_max`      | float | Max speed variation between two rounds(only when increasing) | 0.01 | m/s
`speed_max`    | float | Max linear speed                                          | 0.18  | m/s
`speed_min`    | float |  Min linear speed                                         | 0.05  | m/s
`omega_max`    | float | Max angular velocity                                      | 2.84  | rad/s
`speed_factor` | float | Multiplicative factor applied to computed output speeds   | 0.05  | .
`pid_kp`       | float | PID setting : proportionnal factor                        | 0.5   | .
`pid_ki`       | float | PID setting : integrator factor                           | 0.1   | .
`pid_kp`       | float | PID setting : derivative factor                           | 0     | .

