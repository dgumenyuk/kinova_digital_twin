# kinova_digital_twin
This repository contains codebase to control the Kinova gen3 robot

### Usage

To launch the *kinova_moveit_control* node follow the following instructions.

1. First, you need to set up the kinova ros2 cortex environment.
Follow the instructions in the following repo [kinova_ros2](https://github.com/Kinovarobotics/ros2_kortex). Specifically, run the commands in **Getting started** (instructions for Kinova gen3 7 Dof).

2. Test the installation by executing the command:
```python
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py use_fake_hardware:=True launch_rviz:=True use_sim_time:=True robot_ip:=0.0.0.0
```
You should see a kinova gen3 robot show up in Rviz.

3. Next, you need to build the package to control the robot.  
4. Create a *ros2_ws* folder in your home directory and copy the *src* folder from this repo.
5. Execute the commands:
```python
cd ~/ros2_ws
colcon build --packages-select kinova_moveit_control
source install/setup.bash
```
6. Make sure the command from point **2** is running (ros2 launch ...).
7. Run the command:
```python
ros2 run kinova_moveit_control kinova_moveit_control_node
```