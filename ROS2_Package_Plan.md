# Planned ROS2 Package Directories for Migration

The following ROS2 package directories will be created to migrate the Enhanced_ReKep4xarm_Tinker project from ROS1 to ROS2. Each will be structured as a standalone ROS2 Python package with its own `setup.py` and `package.xml`.

## ROS2 Packages

1. **rekep_ros2**
   - Migrated from: `rekep/` and `main_rekep.py`
   - Function: Planning, environment, and main orchestration

2. **anygrasp_ros2**
   - Migrated from: `anygrasp/detecter_ros.py`
   - Function: Perception (grasp detection)

3. **point_tracker_ros2**
   - Migrated from: `point_tracker/point_track_ros.py`
   - Function: Communication (keypoint/point tracking)

4. **realsense_camera_ros2**
   - Migrated from: `realsense_camera_ros.py`
   - Function: Perception (camera interface)

5. **vision_pipeline_ros2**
   - Migrated from: `vision_pipeline.py`
   - Function: Perception (vision pipeline, e.g., SAM)

## Notes
- Each package will have its own `setup.py`, `package.xml`, and ROS2-compliant node(s).
- Dependencies on ROS1 (`rospy`, ROS1 messages) will be replaced with ROS2 equivalents (`rclpy`, ROS2 messages).
- Launch files and configuration will be updated to ROS2 standards.

---

This list will be updated as migration progresses.
