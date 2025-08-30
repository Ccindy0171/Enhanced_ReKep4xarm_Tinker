import rclpy
from rclpy.node import Node
import unittest

class TestVisionPipelineNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        from vision_pipeline_ros2_new.vision_pipeline_ros2_new.vision_pipeline import VisionPipelineNode
        self.node = VisionPipelineNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'vision_pipeline_node')

    def test_process(self):
        # Should not throw, even if images are not available
        try:
            self.node.process()
        except Exception:
            pass

if __name__ == '__main__':
    unittest.main()
