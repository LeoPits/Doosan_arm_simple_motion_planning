# DOOSAN Robot arm simple motion planning
## (work in progress)


This repository contains the code and documentation for a motion control system for DOOSAN robots. The system  control of the robot's movements,  simple trajectory planning with linear interpolation 
![simplescreenrecorder-2025-01-22_17 23 45-ezgif com-video-to-gif-converter (1)](https://github.com/user-attachments/assets/b6ea01a1-e6ca-4821-9beb-beb53ec150a6)

Launch the file
 ```
roslaunch my_robot_controller m0609_Rviz.launch
 ```

For moving the robot initialize the controller node
 ```
rosrun my_robot_controller robot_controller_node
```



Public the message to change the position and orientation
 ```

rostopic pub /target_position geometry_msgs/Pose "position:
  x: 0.4
  y: 0.2
  z: 0.5
orientation:
  x: 0.4
  y: 0.3
  z: 0.2
  w: 0.1" 
```
Add Marker visualization in rviz simulator with the topic /trajectory_marker to visualize the trajectory 



## System requirements
- ubuntu 20.04 lts
- ros noetic

## Dependencies
https://github.com/doosan-robotics/doosan-robot.git
