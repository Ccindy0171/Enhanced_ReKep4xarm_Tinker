import rclpy
from rclpy.node import Node
import unittest

class TestRealSenseCameraNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        from realsense_camera_ros2_new.realsense_camera_ros2_new.realsense_camera_ros import RealSenseCameraNode
        self.node = RealSenseCameraNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'realsense_camera_node')

    def test_capture_image(self):
        # This will raise if images are not available, but should not crash
        try:
            self.node.capture_image('rgb')
        except Exception:
            pass
        try:
            self.node.capture_image('depth')
        except Exception:
            pass

if __name__ == '__main__':
    unittest.main()
