#!/bin/sh

# Turtlebot startup script
# By Grégoire et Amaury
# Executed at robot startup to speed up deployment when experimenting

service sshd start

ping 10.0.0.1 -w 5 -c 5

ping 10.0.0.2 -w 5 -c 5

date > /home/burger/last_startup