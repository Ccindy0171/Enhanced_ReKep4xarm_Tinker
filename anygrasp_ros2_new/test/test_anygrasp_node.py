import rclpy
from rclpy.node import Node
import unittest
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class TestAnyGraspNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        from anygrasp_ros2_new.anygrasp_ros2_new.detecter_ros import AnyGraspNode
        self.node = AnyGraspNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'grasp_detection_node')

    def test_grasp_callback(self):
        # Simulate a target point message
        msg = Point()
        msg.x, msg.y, msg.z = 0.1, 0.2, 0.3
        try:
            self.node.grasp_callback(msg)
        except Exception as e:
            self.fail(f"Grasp callback failed: {e}")

if __name__ == '__main__':
    unittest.main()
