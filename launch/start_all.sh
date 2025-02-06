#!/bin/bash

gnome-terminal \
 --tab -e "ros2 launch p2-drone-formation-control-simulator multi_sitl_dds_udp.launch.py"  /
gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I0 --model=json --map --sysid=1" /
 sleep 10
 gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I1 --model=json --map --sysid=2" /
 sleep 10
 gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f quad -I2 --model=json --map --sysid=3" /
 gnome-terminal \
 --tab -e "ros2 launch p2-drone-formation-control-simulator mavros_launch.xml " /