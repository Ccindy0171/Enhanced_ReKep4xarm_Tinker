#!/usr/bin/env python3

"""
Simple launch script for ROS2 migrated nodes
This demonstrates how the nodes can be launched together
"""

import subprocess
import time
import signal
import sys
import os

def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking dependencies...")
    
    # Check TAPnet installation
    try:
        import rclpy
        print("✓ ROS2 (rclpy) available")
    except ImportError:
        print("✗ ROS2 (rclpy) not available")
        return False
    
    try:
        from tapnet.torch1 import tapir_model
        print("✓ TAPnet available")
    except ImportError:
        print("✗ TAPnet not available")
        print("Run: python3 tapnet_diagnostic.py")
        print("Or: ./setup_tapnet.sh")
        return False
    
    # Check checkpoint
    checkpoint_path = "checkpoints/causal_bootstapir_checkpoint.pt"
    if os.path.exists(checkpoint_path):
        print("✓ TAPnet checkpoint available")
    else:
        print(f"✗ TAPnet checkpoint not found: {checkpoint_path}")
        print("Run: ./setup_tapnet.sh")
        return False
    
    return True

def launch_nodes():
    """Launch all ROS2 nodes"""
    
    # Check dependencies first
    if not check_dependencies():
        print("\nDependency check failed. Please fix the issues above before launching.")
        return 1
    
    processes = []
    
    try:
        # Launch camera node
        print("Launching camera node...")
        camera_proc = subprocess.Popen([
            sys.executable, 'realsense_camera_ros.py'
        ], cwd='/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')
        processes.append(camera_proc)
        time.sleep(2)
        
        # Launch point tracker node
        print("Launching point tracker node...")
        tracker_proc = subprocess.Popen([
            sys.executable, 'point_tracker/point_track_ros.py'
        ], cwd='/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')
        processes.append(tracker_proc)
        time.sleep(2)
        
        # Launch anygrasp node
        print("Launching anygrasp node...")
        anygrasp_proc = subprocess.Popen([
            sys.executable, 'anygrasp/detecter_ros.py'
        ], cwd='/home/tinker/R2S2R_implementation/Enhanced_ReKep4xarm_Tinker')
        processes.append(anygrasp_proc)
        time.sleep(2)
        
        print("All nodes launched. Press Ctrl+C to stop...")
        
        # Wait for user interrupt
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down nodes...")
        for proc in processes:
            proc.terminate()
        
        # Wait for processes to terminate
        for proc in processes:
            proc.wait()
        
        print("All nodes stopped.")
    
    return 0

if __name__ == '__main__':
    sys.exit(launch_nodes())
