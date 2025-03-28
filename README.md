# Centerpoint-TensorRT-ROS
This repository contains sources and model for CenterPoint inference using TensorRT.

## Environments
To build the pointpillars inference, TensorRT, ROS and CUDA are needed.
 - Ubuntu 20.04
 - NVIDIA Jetson AGX Orin (32GB ram)
 - CUDA: 11.4.239
 - cuDNN: 8.4.1.50
 - TensorRT: 8.4.1.5
 - Jetpack 5.0.2

## Model
The model is created with mmdetection3d. The onnx file can be converted by [onnx_tools](https://github.com/CV-player/mmdetection3d/tree/master/tools/onnx_tools/centerpoint).

Overall inference has five phases:
1. Convert points cloud into 4-channle voxels
2. Extend 4-channel voxels to 10-channel voxel features
3. Run pfe TensorRT engine to get 64-channel voxel features
4. Run rpn backbone TensorRT engine to get 3D-detection raw data
5. Parse bounding box, class type and direction

## Data
The demo used the custom dataset like KITTI.

## Compile && Run

```bash
mkdir centerpoint_ws && cd centerpoint_ws
git clone https://github.com/CV-player/Centerpoint-TensorRT-ROS.git
catkin_make
source devel/setup.bash
roslaunch perception perception.launch
```
