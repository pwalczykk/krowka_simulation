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
    - $ git clone https://github.com/pwalczykk/krowka_simulation.git .

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
## Drag and drop mode with inverse kinematics
1. Starting simulation server
    - $ roslaunch krowka_model krowka_gazebo.launch

2. Starting robot motors controllers and camera
    - $ roslaunch krowka_model krowka_controllers.launch

3. Starting inverse kinematics node
    - $ roslaunch krowka_model krowka_inverse_kinematics.launch

4. Starting trajectory planer
    - $ roscd krowka_model/scripts
    - $ ./run_trajectory.sh

## Teleoperation mode - controlling joints
1. Starting simulation server
    - $ roslaunch krowka_model krowka_gazebo.launch

2. Starting robot motors controllers and camera
    - $ roslaunch krowka_model krowka_controllers.launch

3. Starting teleoperation gui
    - $ roslaunch krowka_model krowka_teleop.launch
