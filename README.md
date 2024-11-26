# p2-drone-formation-control-simulator

- simulates quadcopter drones flying in formation
- drones followes path and maintain the formation
- path and formation can be changed

## Installation
Install according to their documentation
- Ubuntu 24.04 
- ROS2 jazzy jalisco  
- gazeboo harmonic -> sudo apt install ros-jazzy-ros-gz 
- ardupilot:
    1. change to the folder you want to install ardupilot 
    2. mkdir -p ardu_ws/src
        -> need to be installed in own ros workspace        
    3. cd ardu_ws
    4. vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
        -> this downloads the repo
    4. "cd ardu_ws"
        -> next steps need to be done in workspace folder
    5. "sudo apt update" 
    6. "rosdep update"
        -> could be that you need to run "sudo rosdep init" first if rosdep is not initalized before
    7. "source /opt/ros/jazzy/setup.bash"
    8. "rosdep install --from-paths src --ignore-src -r -y"
    9. sudo apt install default-jre
    10. "git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git"
    11. "cd Micro-XRCE-DDS-Gen"
    12. "./gradlew assemble"
        -> failed for me, needed to "sudo apt install openjdk-17-jdk"
        -> "sudo update-alternatives --config java"
        -> and switch to older java version 
    14. "echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc"
    15. "source ~/.bashrc"
    16. "cd .."
        -> change back to workspace folder
    17. "colcon build --packages-up-to ardupilot_dds_tests"
        -> fails because of error fastcdr version in cmake
           "https://github.com/micro-ROS/micro-ROS-Agent"
        -> we removed the downloaded micro-ROS-AGENT folder 
           and downloaded the branch for jazzy directly from github
    18. "source ./install/setup.bash"
    19. "colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+"
        -> to show the result of the test use "colcon test-result --all --verbose"

## Usage
- add later

## Authors and acknowledgment
- add later

## License
- add later

