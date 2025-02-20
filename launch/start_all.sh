#!/bin/bash

gnome-terminal \
 --tab -e "ros2 launch p2-drone-formation-control-simulator multi_sitl_dds_udp.launch.py"  /
gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I0 --model=json --sysid=1" /
 sleep 6
 gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I1 --model=json --sysid=2" /
 sleep 6
 gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I2 --model=json --sysid=3" /
 sleep 6
 gnome-terminal \
 --tab -e "ros2 launch p2-drone-formation-control-simulator mavros_launch.xml " /
 sleep 45
 gnome-terminal \
 --tab -e "ros2 launch p2-drone-formation-control-simulator gui.launch.py  " /


