# Simulation of Krowka robotic arm
Package contains multibody simulation (MBS) of Krowka robotic arm in Gazebo and Robot Operating System (ROS)

# Requirements:
 - ROS with Gazebo, version desktop-full (http://wiki.ros.org/kinetic/Installation)
 - Additional ROS packages required:
    - gazebo_ros_control
    - controller-manager
    - ros-controllers
    - ros-control
    - opencv3
    - cv-bridge
    - image-transport
    - dynamic-reconfigure

# Installation:
1. Copy simulation source code from server
    - $ mkdir -p ~/krowka_simulation/src
    - $ cd ~/krowka_simulation/src
    - $ git clone https://gitlab.com/aghspace/rover-krowka-simulation-ros.git .

2. Install dependencies
    - $ cd ~/krowka_simulation/src
    - $ ./install_deps.sh

3. Compile source code
    - $ cd ~/krowka_simulation
    - $ catkin_make

4. Update .bashrc
    - $ echo "source ~/krowka_simulation/devel/setup.bash --extend" >> ~/.bashrc
    - $ source ~/.bashrc


# Launching:
1. Starting simulation server (choose 1 from 2 below)
    - $ roslaunch krowka_simulation_model start_gazebo_flat.launch (lightweight)
    - $ roslaunch krowka_simulation_model start_gazebo_heightmap.launch (full enviroment)

2. Starting rover motors controllers, loacalization and teleoperation
    - $roslaunch krowka_simulation_model start_platform.launch
