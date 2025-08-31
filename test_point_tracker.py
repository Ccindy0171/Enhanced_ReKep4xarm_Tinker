#!/usr/bin/env python3

"""
Test script for ROS2 migrated point tracker
"""

import rclpy
import sys
import os

# Add the project root to Python path
sys.path.append('/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')

def test_point_tracker():
    """Test the point tracker node"""
    try:
        from point_tracker.point_track_ros import main
        main()
    except Exception as e:
        print(f"Point tracker test failed: {e}")
        return False
    return True

if __name__ == '__main__':
    print("Testing ROS2 migrated point tracker...")
    success = test_point_tracker()
    if success:
        print("Point tracker test completed")
    else:
        print("Point tracker test failed")
