# p2-drone-formation-control-simulator

- simulates quadcopter drones flying in formation
- drones followes path and maintain the formation
- path and formation can be changed

## Installation
Install according to their documentation
- Ubuntu 24.04
- ROS2 jazzy jalisco
- gazeboo harmonic
- ardupilot: 
    1. clone github repo for ardupilot, use main-branch
    2. "cd ardupilot"
    3. "./Tools/environment_install/install-prereqs-ubuntu.sh -y"
        -> can take some time, because dependencies are downloaded and 
           built if necessary
    4. "./waf configure --board sitl"
        -> I had problems with python future module
        -> "sudo apt install python3-future"
    5. "./waf copter"
    6. " ./Tools/autotest/sim_vehicle.py -v copter --console --map -w"
    -> this should start the sitl
    -> but so far nothing happens in the simulation

## Usage
- add later

## Authors and acknowledgment
- add later

## License
- add later

