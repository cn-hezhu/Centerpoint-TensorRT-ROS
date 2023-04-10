# Centerpoint-TensorRT-ROS
# Model && Data
## Model
This repository contains sources and model for CenterPoint inference using TensorRT. The model is created with mmdetection3d.

Overall inference has five phases:

Convert points cloud into 4-channle voxels
Extend 4-channel voxels to 10-channel voxel features
Run pfe TensorRT engine to get 64-channel voxel features
Run rpn backbone TensorRT engine to get 3D-detection raw data
Parse bounding box, class type and direction
## Data
The demo used the custom dataset like KITTI.

# Environments
To build the pointpillars inference, TensorRT, ROS and CUDA are needed.
 - Ubuntu 20.04
 - NVIDIA Jetson AGX Orin (32GB ram)
 - CUDA: 11.4.239
 - cuDNN: 8.4.1.50
 - TensorRT: 8.4.1.5
 - Jetpack 5.0.2
## Compile && Run

```bash
mkdir centerpoint_ws && cd centerpoint_ws
git clone https://github.com/CV-player/Centerpoint-TensorRT-ROS.git
catkin_make
source devel/setup.bash
roslaunch perception perception.launch
```
