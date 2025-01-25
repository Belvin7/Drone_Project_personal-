#!/bin/bash

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -I2" \