#!/usr/bin/env python3

"""
Test script for ROS2 migrated anygrasp node
"""

import rclpy
import sys
import os

# Add the project root to Python path
sys.path.append('/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')

def test_anygrasp_node():
    """Test the anygrasp detection node"""
    try:
        from anygrasp.detecter_ros import main
        main()
    except Exception as e:
        print(f"AnyGrasp node test failed: {e}")
        return False
    return True

if __name__ == '__main__':
    print("Testing ROS2 migrated anygrasp node...")
    success = test_anygrasp_node()
    if success:
        print("AnyGrasp node test completed")
    else:
        print("AnyGrasp node test failed")
