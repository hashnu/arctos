**Alpha version of ros2_control + moveit for Arctos 2.9.5**

Very early version with hardcoded gear ratio, speed and acceleration in arctos_hardware.cpp. Only Position Joint trajectory available.

Has been tested on Ros Jazzy Ubuntu 24.04 only.

Can communication need to have [Canary cpp]([opentibiabr/canary: Canary Server 13.x for OpenTibia community.](https://github.com/opentibiabr/canary)) installed.

Create a new workspace, clone the repository in src folder and run

```
colcon build --packages-select arctos arctos_moveit
```

After sourcing install/setup.bash, set zero position using the helper script

```
ros2 run arctos set_joints_zero_positions.py
```

Ros2_Control, MoveIt and Rviz can be launched by

```
ros2 launch arctos_moveit arctos_moveit.launch.py
```
