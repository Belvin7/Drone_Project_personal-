# p2-drone-formation-control-simulator

## Table of Contents 
- [Main Goal](#main-goal)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [start simulation](#start-simulation)
- [Future work to do](#future-work-to-do)
- [Resources](#resources)
- [Documentation](#documentation)
- [License](#license)
- [Authors](#authors)
- [Misc](#misc)

## Main Goal 
- simulates quadcopter drones flying in formation
- drones follows path and maintain the formation
- path and formation can be changed


![Drones in Formation](resource/Drones_in_Formation.jpeg)

## Prerequisites 

Before proceeding with the installation, ensure you have the following:  
- **Hardware Requirements:** None for the simulation   
- **Operating System:**  Ubuntu 22.04 , Use aif you use a virtual machine if you have windows  
- **Required Libraries:**  ROS2 jazzy jalisco , MAVROS, ArduPilot ,Gazebo 
  
## Installation

Install according to their documentation
- Ubuntu 24.04 
- ROS2 jazzy jalisco  
- gazeboo harmonic -> sudo apt install ros-jazzy-ros-gz 
- tkinter \
    -> rosdep did not find it, if not already installed do so
- ardupilot:
    1. change to the folder you want to install ardupilot 
    2. mkdir -p ardu_ws/src \
        -> need to be installed in own ros workspace 
    3. cd ardu_ws
    4. vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
        -> this downloads the repo
    4. "cd ardu_ws" \
        -> next steps need to be done in workspace folder
    5. "sudo apt update" 
    6. "rosdep update" \
        -> could be that you need to run "sudo rosdep init" first if rosdep is not initalized before
    7. "source /opt/ros/jazzy/setup.bash"
    8. "rosdep install --from-paths src --ignore-src -r -y"
    9. sudo apt install default-jre
    10. "git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git"
    11. "cd Micro-XRCE-DDS-Gen"
    12. "./gradlew assemble" \
        -> failed for me, needed to "sudo apt install openjdk-17-jdk" \
        -> "sudo update-alternatives --config java" \
        -> and switch to older java version 
    14. "echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc"
    15. "source ~/.bashrc"
    16. "cd .." \
        -> change back to workspace folder
    17. "colcon build --packages-up-to ardupilot_dds_tests" \
        -> fails because of error fastcdr version in cmake
           "https://github.com/micro-ROS/micro-ROS-Agent" \
        -> we removed the downloaded micro-ROS-AGENT folder 
           and downloaded the branch for jazzy directly from github
    18. "source ./install/setup.bash"
    19. "colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+" \
        -> to show the result of the test use "colcon test-result --all --verbose" \
        -> next the steps to install necessary packages for gazebo+ros+ardu working together  
    20. "vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src" \
        -> this downloads needed repos \
        -> it also downloads a second micro-ros-agent repo which you need to remove  \
        -> also ros_gz repo is for humble so it didn't compile for me \
        -> you can download the jazzy branch from "https://github.com/gazebosim/ros_gz/tree/jazzy" 
    21. add "export GZ_VERSION=harmonic" to ~/.bashrc
    22. "source /opt/ros/humble/setup.bash"
    23. "sudo apt update"
    24. "rosdep update"
    25. "rosdep install --from-paths src --ignore-src -r"    
    26. "colcon build" (in workspace directory) \
        -> I had a problem because I didn't had enough ram and swap space  \
        -> so if you have a crappy laptop like me (8GB RAM and 4GB swap) you should increase your swap space otherwise 
           your laptop freezes during building 
    27. "colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup" \
        -> test all packages, in this step I got the linter errors \
        -> "colcon test-result --all --verbose" gives the verbose test output
    28. "ros2 launch ardupilot_gz_bringup iris_runway.launch.py" \
        -> now this should be running and you should see a drone standing on a runway \
        -> so far no drone movement but you should see a  window for rviz and  one for gazebo  \
        -> there are more examples on the ardupilot docu site 

- found other package that not work correctly:
    - replace "sdformat_urdf" with jazzy branch "https://github.com/ros/sdformat_urdf/tree/jazzy" 
    - package "ardupilot_gz" also  

29. Install MAVROS with "sudo apt install ros-jazzy-mavros"
30.  Install Geographic lib
    -> ros2 run mavros install_geographiclib_datasets.sh

    # Alternative:
    wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh

31. Add these to your .bashrc to source the models created in the project for Gazebo
    export GZ_SIM_SYSTEM_PLUGIN_PATH=:/home/"your path to the project"/src/p2-drone-formation-control-simulator/models:
    export GZ_SIM_RESOURCE_PATH=:/home/"your path to the project"/src/p2-drone-formation-control-simulator/models:

## Start simulation

1. cd to ardu_ws directory 
2. `colcon build --packages-select p2-drone-formation-control-simulator`
3. `source ./install/setup.bash`
4. start the script : `. src/p2-drone-formation-control-simulator/launch/start_all.sh`

## Future work to do 

- if you know there are things you did not finish or cannot finish in the span of your project mention them here
- could also be a place for "known bugs" (could also be its own section)
- More and different formations
- Adding more complex path-planning
- modify the movement to make it more accurate
- different simulated environments
- Rviz/Robot state publisher

## Resources

- [ArduPilot Documentation](https://ardupilot.org/ardupilot/) – *ArduPilot Development Team* (Last updated: 17.03.2025)
- [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html) – *Open Robotics* (Last updated: 2025)
- [ROS 2 Beginner CLI Tools Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html) – *Open Robotics* (Last updated: 2025)
- [Gazebo Tutorials](https://gazebosim.org/docs/latest/tutorials/) – *Open Source Robotics Foundation* (Last updated: 2025)
- [MAVROS Documentation](http://wiki.ros.org/mavros) – *Open Robotics* (Last updated: 03.03.2018)
- [Tkinter Documentation](https://docs.python.org/3/library/tkinter.html) – *Python Software Foundation* (Last updated: 18.03.2025)

## Documentation

- you can link to your report here

## Authors

- Michael Faber : michael.faber@ovgu.de
- Christian Grüneberg: christian.grueneberg@ovgu.de
- Belvin Benny Thomas : belvin.benny@st.ovgu.de

