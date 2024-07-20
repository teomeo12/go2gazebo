## GO2 Description
This repository contains the urdf model of go2 prepared to be ran an used in ROS2

## Run the library

### 1. Add to your ros2 workspace's source directory
### 2. Build the package

```bash
colcon build
```

### 3. Test the package


```bash
ros2 launch go2_description go2_rviz.launch.py 
```

## When used for isaac gym or other similiar engine 

Collision parameters in urdf can be amended to better train the robot:

Open "go2_description.urdf" in "./go2_description/urdf",
and amend the ` box size="0.213 0.0245 0.034" ` in links of "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh".

For example, change previous values to ` box size="0.11 0.0245 0.034" ` means the length of the thigh is shortened from 0.213 to 0.11, which can avoid unnecessary collision between the thigh link and the calf link. 

The collision model before and after the above amendment are shown as "Normal_collision_model.png" and "Amended_collision_model.png" respectively.
