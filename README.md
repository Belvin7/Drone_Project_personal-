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

## start simulation

- to prevent clutter I made a separate folder where we start the terminals mentioned in the steps below
- because both commands create additional files and a folder 
- to run a single drone simulation:
    1. open a terminal and launch "ros2 launch ardupilot_gz_bringup iris_runway.launch.py" \
        -> now gazebo and rviz should start
    2. open second terminal and run "mavproxy.py --master=udp:127.0.0.1:14550  --console --map --sitl=127.0.0.1:5501" \
        -> this will launch the mav proxy which acts as a ground station for the drone \
        -> maybe "--console" and "--map" not necessary because the additional windows are not really needed 
    3. to start the drone type in the terminal open in step 2. \
        -> "mode guided" \
        -> "arm throttle" \
        -> "takeoff 40" \
        -> now the drone should start the motors and take off to an altitude of 40m \
        -> you should see this in gazebo and rviz 
    4. you can give additional commands \
        -> see "https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html" \
           and "https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html" \
           for further information
    5. "mode rtl" \
        -> drone return to home-waypoint, lands and stops the motors

## Usage
- add later

## Authors and acknowledgment
- add later

## License
- add later

