import rclpy
from rclpy.node import Node
import unittest
from std_msgs.msg import Int32MultiArray

class TestPointTrackerNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        from point_tracker_ros2_new.point_tracker_ros2_new.point_track_ros import PointTrackerNode
        self.node = PointTrackerNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'point_tracking_node')

    def test_point_callback(self):
        # Simulate a tracking points message
        msg = Int32MultiArray()
        msg.data = [0, 100, 100, 1, 200, 200]
        try:
            self.node.point_callback(msg)
        except Exception as e:
            self.fail(f"Point callback failed: {e}")

if __name__ == '__main__':
    unittest.main()
