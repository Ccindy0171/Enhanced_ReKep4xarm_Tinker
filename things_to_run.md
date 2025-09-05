## realsense
```
cindy@cindy-IdeaPad-Linux:~/Documents/R2S2R/realsense-ros$ ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true rgb_camera.color_profile:=1280x720x15 depth_module.depth_profile:=1280x720x15
```

## SoM
```
export all_proxy=''
export ALL_PROXY=''
export OPENAI_API_KEY='hk....'
(SAM2) cindy@cindy-IdeaPad-Linux:~/Documents/R2S2R/Enhanced_ReKep4xarm_Tinker/SoM$ python som_gpt4v/main_som.py grasp
```
对准之后直接按回车，等待结果，然后再跑rekep main

## AnyGrasp
```
(AnyGrasp) cindy@cindy-IdeaPad-Linux:~/Documents/R2S2R/anygrasp_sdk/grasp_detection$ python detector_ros.py 
```

## TAPNet
```
(TAPNet) cindy@cindy-IdeaPad-Linux:~/Documents/R2S2R/Enhanced_ReKep4xarm_Tinker/point_tracker$ python point_track_ros.py 
```


## ReKep
```
export all_proxy=''
export ALL_PROXY=''
export OPENAI_API_KEY='hk....'
(Rekep) cindy@cindy-IdeaPad-Linux:~/Documents/R2S2R/Enhanced_ReKep4xarm_Tinker$ python main_rekep.py
```