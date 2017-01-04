@project:   Project-Gripper
@author:    Przemek Walczyk
@email:     pwalczykk@gmail.com

@type:      Gazebo URDF model

@description:

Model of 3-DOF, Krowka robotic arm

All links beside gripper claws are controlled with PID controllers using position as input.
Manipilator's claws are controlled by force, becosue this way provides more stable grip.

It contains:
    - Kinematic description of manipulator and some basic properties (krowka.xacro)
    - Dynamic description and advanced properties of each link (1_base/ && 2_links/ && 3_gripper/)
    - Included plugins (0_utils/)
    - Geometry of each link in STL format (mesh/)
    - Configuration of PID controllers plugins (yaml/)
