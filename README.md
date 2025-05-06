# Enhanced Rekep for xArm6

## Introduction
[中文README](https://github.com/youngfriday/Enhanced_ReKep4xarm/blob/master/README_CHN.md)

This project is an improvement based on [Rekep](https://github.com/huangwl18/ReKep) and [Koch_VLM_Benchmarks](https://github.com/Quest2GM/Koch_VLM_Benchmarks), which is added some new features and specifically deployed on the [xArm6](https://www.ufactory.cc/xarm-collaborative-robot/) robotic arm.

## TODO&CHANGED List

The following changes and additions are based on the original repository. The code will be updated gradually (expected completion by June). **Stay tuned!**

- [x] Added [SoM](https://github.com/microsoft/SoM) to pre-filter target object masks to reduce the number of keypoints.

- [x] Added [Tapnet](https://github.com/google-deepmind/tapnet) to track keypoints.

- [x] Integrated the grasp detection model module [Anygrasp](https://github.com/graspnet/anygrasp_sdk).

- [ ] Use [GenPose++](https://github.com/Omni6DPose/GenPose2) to enhance key vector tracking, referencing the [Omnimanip](https://github.com/pmj110119/OmniManip) paper. Some code can be referenced from my other repository [GenPose2-SAM-2-real-time](https://github.com/youngfriday/GenPose2-SAM-2-real-time).

- [ ] Use [SAM2]() for mask tracking.

## A Simple Video

- TASK Instruction: Pick up the red block and drop it into the box
- PS: To save time and computational resources, the `path solver` was not used in this video, so the 2nd transition process appears somewhat unnatural.

![demo](https://github.com/youngfriday/Enhanced_ReKep4xarm/blob/master/demo.mp4)

## To Prepare

- **Communication Tool:** To meet the requirements of each module’s environment as closely as possible, communication uses [ROS](https://www.ros.org/), especially for transmitting images between multiple modules.
- **xArm6 Controller**: this project uses the [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK).
- **RGB-D Camera:** Realsense D435i, using [realsense-ros](https://github.com/IntelRealSense/realsense-ros) to publish images.
- **System & GPU:** The system currently runs on an Ubuntu 20.04 machine with an Nvidia 3080 GPU (10GB). To prevent GPU memory overflow, I have opted to run the [SoM] section separately in advance.
- **Environment:** The environment for each submodule should strictly follow the original project’s configuration, and all modules will run separately.
- **CUDA:** 12.1

## References

- [Rekep](https://github.com/huangwl18/ReKep)
- [Koch_VLM_Benchmarks](https://github.com/Quest2GM/Koch_VLM_Benchmarks)
- [Omnimanip](https://github.com/pmj110119/OmniManip)
- [Copa](https://github.com/HaoxuHuang/copa)