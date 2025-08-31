# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Live Demo for PyTorch Online TAPIR."""

import time

import cv2
import numpy as np

from tapnet.torch1 import tapir_model
import torch
import torch.nn.functional as F
import tree
import pyrealsense2 as rs
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NUM_POINTS = 8

class PointTrackerNode(Node):
    def __init__(self):
        super().__init__('point_tracking_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize tracking variables
        self.rgb_frame = None
        self.first_frame_received = False
        self.have_point = [False] * NUM_POINTS
        self.point_idx = []
        self.query_frame = True
        
        # Setup device
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
            
        # Load and initialize model
        self.load_model()
        
        # Initialize query points and state
        self.query_points = torch.zeros([NUM_POINTS, 3], dtype=torch.float32).to(self.device)
        self.query_features = None
        self.causal_state = None
        
        # Create publishers
        self.tracking_points_pub = self.create_publisher(
            Int32MultiArray, '/current_tracking_points', 10)
        
        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10)
        self.points_sub = self.create_subscription(
            Int32MultiArray, '/tracking_points', self.point_callback, 10)
        
        self.get_logger().info("PointTrackerNode initialized")
        
    def load_model(self):
        """Load and initialize the TAPIR model"""
        self.get_logger().info("Creating model...")
        model = tapir_model.TAPIR(pyramid_level=1, use_casual_conv=True)
        self.get_logger().info("Loading checkpoint...")
        model.load_state_dict(
            torch.load("checkpoints/causal_bootstapir_checkpoint.pt")
        )
        model = model.to(self.device)
        model = model.eval()
        torch.set_grad_enabled(False)
        self.model = model
        
    def preprocess_frames(self, frames):
        """Preprocess frames to model inputs."""
        frames = frames.float()
        frames = frames / 255 * 2 - 1
        return frames
        
    def online_model_init(self, frames, points):
        """Initialize query features for the query points."""
        frames = self.preprocess_frames(frames)
        feature_grids = self.model.get_feature_grids(frames, is_training=False)
        features = self.model.get_query_features(
            frames,
            is_training=False,
            query_points=points,
            feature_grids=feature_grids,
        )
        return features
        
    def postprocess_occlusions(self, occlusions, expected_dist):
        """Process occlusion predictions"""
        visibles = (1 - F.sigmoid(occlusions)) * (1 - F.sigmoid(expected_dist)) > 0.5
        return visibles
        
    def online_model_predict(self, frames, features, causal_context):
        """Compute point tracks and occlusions given frames and query points."""
        frames = self.preprocess_frames(frames)
        feature_grids = self.model.get_feature_grids(frames, is_training=False)
        trajectories = self.model.estimate_trajectories(
            frames.shape[-3:-1],
            is_training=False,
            feature_grids=feature_grids,
            query_features=features,
            query_points_in_video=None,
            query_chunk_size=64,
            causal_context=causal_context,
            get_causal_context=True,
        )
        causal_context = trajectories["causal_context"]
        del trajectories["causal_context"]
        tracks = trajectories["tracks"][-1]
        occlusions = trajectories["occlusion"][-1]
        uncertainty = trajectories["expected_dist"][-1]
        visibles = self.postprocess_occlusions(occlusions, uncertainty)
        return tracks, visibles, causal_context
        
    def image_callback(self, msg):
        """Process received image messages and convert to numpy array, crop to square."""
        # self.get_logger().debug("Received image data.")
        self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.first_frame_received = True
        
        # Crop image to square
        trunc = abs(self.rgb_frame.shape[1] - self.rgb_frame.shape[0]) // 2
        if self.rgb_frame.shape[1] > self.rgb_frame.shape[0]:
            self.rgb_frame = self.rgb_frame[:, trunc:-trunc]
        elif self.rgb_frame.shape[1] < self.rgb_frame.shape[0]:
            self.rgb_frame = self.rgb_frame[trunc:-trunc]
            
        # Initialize model on first frame
        if self.query_features is None and self.first_frame_received:
            frame = torch.tensor(self.rgb_frame).to(self.device)
            self.query_features = self.online_model_init(
                frames=frame[None, None], points=self.query_points[None, :]
            )
            self.causal_state = self.model.construct_initial_causal_state(
                NUM_POINTS, len(self.query_features.resolutions) - 1
            )
            self.causal_state = tree.map_structure(lambda x: x.to(self.device), self.causal_state)
            
    def point_callback(self, msg):
        """Handle tracking point information from external source."""
        points = np.zeros((NUM_POINTS, 3), dtype=np.float32)
        data = np.array(msg.data).reshape(-1, 3)
        self.point_idx = []
        num = 0
        
        for point in data:
            idx, x, y = point
            self.point_idx.append(int(idx))
            x = x - (1280 - 720) / 2  # Adjust x coordinate
            pos = (y, x)
            points[num] = np.array((0,) + pos, dtype=np.float32)
            self.get_logger().info(f"Received point {int(idx)}: ({x}, {y})")
            self.have_point[num] = True
            num += 1
            
        self.query_frame = True
        self.track_points(num, points)
        
    def track_points(self, num, points):
        """Main tracking loop"""
        cv2.namedWindow("Point Tracking")
        
        with torch.no_grad():
            while self.rgb_frame is not None and rclpy.ok():
                frame = self.rgb_frame
                numpy_frame = frame.copy()
                
                if self.query_frame and self.query_features is not None:
                    frame = torch.tensor(frame).to(self.device)
                    for i in range(num):
                        query_points_i = torch.tensor(points[i]).to(self.device)
                        init_query_features = self.online_model_init(
                            frames=frame[None, None], points=query_points_i[None, None]
                        )
                        self.query_features, self.causal_state = self.model.update_query_features(
                            query_features=self.query_features,
                            new_query_features=init_query_features,
                            idx_to_update=np.array([int(i)]),  
                            causal_state=self.causal_state,
                        )
                self.query_frame = False

                if self.query_features is not None and self.causal_state is not None:
                    frame = torch.tensor(frame).to(self.device)
                    track, visible, self.causal_state = self.online_model_predict(
                        frames=frame[None, None],
                        features=self.query_features,
                        causal_context=self.causal_state,
                    )
                    track = np.round(track.cpu().numpy())
                    visible = visible.cpu().numpy()
                    
                    tracked_points = []
                    for i, _ in enumerate(self.have_point):
                        if visible[0, i, 0] and self.have_point[i] and i < len(self.point_idx):
                            x, y = int(track[0, i, 0, 0]), int(track[0, i, 0, 1])
                            tracked_points.append((int(self.point_idx[i]), int(x + (1280 - 720) / 2), int(y)))
                            cv2.circle(numpy_frame, (x, y), 5, (255, 0, 0), -1)

                    # Convert tracked point positions to ROS message and publish
                    if tracked_points:
                        msg_to_send = Int32MultiArray()
                        msg_to_send.data = [item for sublist in tracked_points for item in sublist]
                        self.tracking_points_pub.publish(msg_to_send)

                cv2.imshow("Point Tracking", numpy_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.001)

def main(args=None):
    """Main function to run the point tracker node"""
    print("Welcome to the TAPIR PyTorch live demo.")
    print("Please note that if the framerate is low (<~12 fps), TAPIR performance")
    print("may degrade and you may need a more powerful GPU.")
    
    rclpy.init(args=args)
    
    try:
        node = PointTrackerNode()
        
        # Wait for first frame and tracking points
        node.get_logger().info("Waiting for tracking point messages...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()