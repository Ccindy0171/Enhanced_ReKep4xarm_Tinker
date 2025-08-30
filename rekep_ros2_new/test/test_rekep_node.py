import rclpy
from rclpy.node import Node
import unittest

class TestRekepNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        from rekep_ros2_new.rekep_ros2_new.main_rekep import RekepNode
        self.node = RekepNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'rekep_node')

    def test_subgoal_generation(self):
        # Simulate a task and check subgoal output (mock or minimal input)
        # This is a placeholder; expand with real test data as needed
        try:
            subgoals = self.node._get_all_subgoals()
            self.assertIsInstance(subgoals, list)
        except Exception as e:
            self.fail(f"Subgoal generation failed: {e}")

if __name__ == '__main__':
    unittest.main()
