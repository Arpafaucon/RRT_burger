## Robot Setup

We noticed that in some cases, the turtlebot struggles to setup its network connections and stays unreachable.

The script below acts as a workaround:

#### startup_robot.sh
```sh
#!/bin/sh

# Turtlebot startup script
# By Grégoire et Amaury
# Executed at robot startup to speed up deployment when experimenting

# Starting the ssh server service
service sshd start

# Trying to ping the router
ping 10.0.0.1 -w 5 -c 5

# Trying to ping the desktop PC salade
ping 10.0.0.2 -w 5 -c 5

# To check if the script runned well
date > /home/burger/last_startup
```