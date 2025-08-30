# Enhanced_ReKep4xarm_Tinker ROS2 Migration Documentation

## Overview
This repository has been fully migrated from ROS1 to ROS2. All major nodes and modules have been ported to use ROS2 (`rclpy`) APIs, and the codebase is now compatible with ROS2 build and runtime environments.

## Migrated ROS2 Packages
- **rekep_ros2_new**: Main orchestration and task logic
- **anygrasp_ros2_new**: Grasp detection and proposal
- **point_tracker_ros2_new**: Point tracking using deep learning
- **realsense_camera_ros2_new**: RealSense camera interface and utilities
- **vision_pipeline_ros2_new**: Vision pipeline for segmentation and tracking

## Key Migration Changes
- All nodes now use `rclpy` and ROS2 node structure
- Publishers, subscribers, and message types updated for ROS2
- ROS1-specific code and imports removed
- Each package contains a ROS2-compliant entrypoint and can be launched as a ROS2 node

## Build Instructions
1. Ensure all Python dependencies are installed in your ROS2 environment (see requirements in each package)
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
4. Run nodes using `ros2 run <package> <executable>`


## Configuration Details

### 1. Camera Intrinsic Matrix and Extrinsics

- **Intrinsic Matrix (K):**
   The RealSense camera uses the following intrinsic matrix (in `realsense_camera_ros.py` and related nodes):
   ```
   K = [
      [908.94415283, 0, 641.31561279],
      [0, 908.80529785, 370.88174438],
      [0, 0, 1]
   ]
   ```
   - This matrix is used for pixel-to-3D point conversion and camera calibration.
   - The distortion coefficients are set to zero by default: `D = [0, 0, 0, 0, 0]`.

- **Extrinsic Matrix:**
   - The file `camera_extrinsic1.npy` contains a 4x4 transformation matrix (rotation and translation) from the camera to the world or robot base frame.
   - This is loaded at runtime by the camera and vision nodes.
   - If the file is missing or malformed, the system falls back to the identity matrix.

- **Camera Info Topic:**
   - The node subscribes to `/camera/color/camera_info` to update the intrinsic matrix at runtime if available.

### 2. rekep/configs/config.yaml

- **Main Parameters:**
   - `interpolate_pos_step_size`, `interpolate_rot_step_size`: Control path density and rotation granularity.
   - `grasp_depth`, `constraint_tolerance`, `bounds_min`, `bounds_max`: Workspace and planning constraints.
   - `sdf_voxel_size`: Voxel size for signed distance field.
   - `action_steps_per_iter`: Number of action steps per iteration.

- **Environment:**
   - `video_cache_size`, `physics_frequency`, `action_frequency`: Simulation and environment settings.
   - `scene`: Scene name, type, and model.
   - `robot`: Robot configuration, including controller types and parameters.

- **Camera Configuration:**
   - Multiple cameras can be defined (see `camera:` section), each with:
      - `name`, `position` (3D), `orientation` (quaternion), `resolution`.

- **Path Solver, Subgoal Solver, Keypoint Proposer:**
   - Each has its own step sizes, bounds, and optimization settings.
   - `constraint_generator`: Model and parameters for constraint generation (e.g., GPT-4o).

### 3. SoM/grasp/grasp_cfg.yaml

- **Grasp Task Configuration:**
   - `image`: Path to the input image for grasping.
   - `task`: Task description (e.g., "pick up the red block and drop it into the box").
   - `granularity1`, `granularity2`: Granularity parameters for mask or grasp generation.
   - `output`: Output file for the generated mask.
   - `response`: Output file for grasp response.
   - `filter`: Filtering options for mask/grasp selection, including area and intersection thresholds.

### 4. sam2/configs/sam2.1/sam2.1_hiera_l.yaml

- **SAM2 Model Configuration:**
   - Defines the architecture for the SAM2 segmentation model.
   - Includes encoder/decoder settings, backbone structure, attention layers, and position encoding.
   - Used by the vision pipeline for segmentation tasks.

### 5. General Notes

- **All configuration files are in YAML or NPY (NumPy) format.**
- **Update paths as needed** if you move configuration files or data.
- **For custom tasks or environments, edit the relevant YAML files** in `rekep/configs/`, `SoM/grasp/`, or `sam2/configs/`.
- **Model checkpoints** (e.g., for SAM2 or Cutie) must be downloaded and placed in the correct directories as referenced in the configs.

----

# Test Plan for Each Module

## rekep_ros2_new
- **Test:** Launch the node and verify it initializes, subscribes, and publishes as expected.
- **Test:** Simulate task requests and check for correct subgoal and path planning outputs.
- **Test:** Validate error handling for missing or malformed input.

## anygrasp_ros2_new
- **Test:** Publish a sample RGB and depth image, and a target point. Verify the node publishes a valid grasp pose.
- **Test:** Check that the node handles missing images gracefully.
- **Test:** Validate grasp pose output for known test images.

## point_tracker_ros2_new
- **Test:** Publish a sample image and tracking points. Verify the node publishes tracked points and visualizes them.
- **Test:** Check for correct handling of missing or malformed tracking data.

## realsense_camera_ros2_new
- **Test:** Launch the node and verify it subscribes to camera topics and processes images.
- **Test:** Validate the output of camera calibration and 3D point conversion functions.
- **Test:** Check for correct error handling when images are not available.

## vision_pipeline_ros2_new
- **Test:** Launch the node and verify the vision pipeline initializes and processes images.
- **Test:** Provide a sample image and mask, and check the segmentation and tracking outputs.
- **Test:** Validate error handling for missing or invalid input.

---

## Notes
- Ensure all dependencies (Python packages, model checkpoints, etc.) are installed and available in your ROS2 environment.
- For integration tests, use ROS2 launch files and sample data to simulate real-world scenarios.
- For more details, see the README.md in each package and the main project README.
