#!/usr/bin/env python3

"""
Test script for ROS2 migrated camera node
"""

import rclpy
import sys
import os

# Add the project root to Python path
sys.path.append('/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')

def test_camera_node():
    """Test the camera node"""
    try:
        from realsense_camera_ros import main
        main()
    except Exception as e:
        print(f"Camera node test failed: {e}")
        return False
    return True

if __name__ == '__main__':
    print("Testing ROS2 migrated camera node...")
    success = test_camera_node()
    if success:
        print("Camera node test completed")
    else:
        print("Camera node test failed")
