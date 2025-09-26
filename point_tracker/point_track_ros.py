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

from tapnet.torch import tapir_model
import torch
import torch.nn.functional as F
import tree
import pyrealsense2 as rs
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

NUM_POINTS = 8

class PointTrackerNode(Node):
    def __init__(self, debug=False):
        super().__init__('point_tracking_node')
        
        self.debug = debug
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize tracking variables
        self.rgb_frame = None
        self.first_frame_received = False
        self.have_point = [False] * NUM_POINTS
        self.point_idx = []
        self.query_frame = True
        self.next_query_idx = 0

        # Reentrant group so timer + subs can run concurrently
        self.cb_group = ReentrantCallbackGroup()
        
        # # Setup device
        # if torch.cuda.is_available():
        #     self.device = torch.device("cuda")
        # else:
        #     self.device = torch.device("cpu")
        self.device = torch.device('cpu')
            
        # Load and initialize model
        self.load_model()
        
        # Initialize query features and state - will be set properly in image_callback
        self.query_features = None
        self.causal_state = None
        
        # Create publishers
        self.tracking_points_pub = self.create_publisher(
            Int32MultiArray, 
            '/current_tracking_points', 
            10
            )
        
        self.rgb_frame = None
        self.frame_lock = threading.Lock()
        self.tracking_timer = None                   # NEW: timer handle
        self._tracking_active = False                # preserve existing flag
        self.step_lock = threading.Lock()
        self.add_points_queue = []
        self.points_queue_lock = threading.Lock()
        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 
            qos_profile_sensor_data,
            callback_group=self.cb_group)
        self.points_sub = self.create_subscription(
            Int32MultiArray, '/tracking_points', self.point_callback, 10, callback_group=self.cb_group)
        
        
        # # Send debug test message if debug mode is enabled
        # if self.debug:
        #     self.debug_timer = self.create_timer(2.0, self.send_debug_message)  # Send test message after 2 seconds
        #     self.debug_pub = self.create_publisher(Int32MultiArray, '/tracking_points', 10)
        #     self.get_logger().info("Debug mode enabled: will send test tracking points.")

        self.get_logger().info(f"PointTrackerNode initialized (debug={self.debug})")
        
    def send_debug_message(self):
        """Send a test message to the tracking points topic for debugging"""
        test_msg = Int32MultiArray()
        # Send test tracking points: point 1 at (400, 300), point 2 at (600, 400)
        test_msg.data = [1, 400, 300, 2, 600, 400]
        self.debug_pub.publish(test_msg)
        self.get_logger().info("Sent debug test message: [1, 400, 300, 2, 600, 400]")
        
        # Cancel the timer after sending the message once
        self.destroy_timer(self.debug_timer)
        
    def load_model(self):
        """Load and initialize the TAPIR model"""
        self.get_logger().info("Creating model...")
        model = tapir_model.TAPIR(pyramid_level=1, use_casual_conv=True)
        self.get_logger().info("Loading checkpoint...")
        # get directory of this python file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model.load_state_dict(
            # load with absolute path or ensure the checkpoint is in the working directory
            torch.load(os.path.join(dir_path, "checkpoints/causal_bootstapir_checkpoint.pt"))
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
        
    # def image_callback(self, msg):
    #     """Process received image messages and convert to numpy array, crop to square."""
    #     # self.get_logger().debug("Received image data.")
    #     self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    #     # Crop image to square
    #     trunc = abs(self.rgb_frame.shape[1] - self.rgb_frame.shape[0]) // 2
    #     if self.rgb_frame.shape[1] > self.rgb_frame.shape[0]:
    #         self.rgb_frame = self.rgb_frame[:, trunc:-trunc]
    #     elif self.rgb_frame.shape[1] < self.rgb_frame.shape[0]:
    #         self.rgb_frame = self.rgb_frame[trunc:-trunc]
            
    #     # Initialize model on first frame - DO NOT initialize query features here
    #     # Just prepare for future initialization when we get tracking points
    #     if self.query_features is None and not self.first_frame_received:
    #         # Only initialize causal state structure, not query features
    #         self.get_logger().info("First frame received, ready for tracking points")
    #         self.first_frame_received = True  # Only need to log once
    def image_callback(self, msg):
        """Process received image messages and convert to numpy array, crop to square."""
        self.get_logger().debug("Received image data.")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Crop image to square
        trunc = abs(frame.shape[1] - frame.shape[0]) // 2
        if frame.shape[1] > frame.shape[0]:
            frame = frame[:, trunc:-trunc]
        elif frame.shape[1] < frame.shape[0]:
            frame = frame[trunc:-trunc]

        # Store atomically
        acquired = self.frame_lock.acquire(timeout=1.0)
        if not acquired:
            self.get_logger().warning("Failed to acquire frame lock in image callback")
            return
        else:
            self.rgb_frame = frame
            self.frame_lock.release()

        if self.query_features is None and not self.first_frame_received:
            self.get_logger().info("First frame received, ready for tracking points")
            self.first_frame_received = True

            
    def point_callback(self, msg):
        """Handle tracking point information from external source."""
        self.get_logger().info(f"Received tracking point message: {msg}.")
        data = np.array(msg.data).reshape(-1, 3)
        self.get_logger().info(f"Received tracking points: {data}")
        self.point_idx = []
        
        if len(data) > NUM_POINTS:
            self.get_logger().warning(f"Received more than {NUM_POINTS} points, only using the first {NUM_POINTS}.")
            data = data[:NUM_POINTS]
        
        # Process each point and add to tracking
        for i, point in enumerate(data):
            idx, x, y = point
            self.point_idx.append(int(idx))
            # Adjust x coordinate for cropping
            x_adjusted = x - (1280 - 720) / 2
            self.get_logger().info(f"Received point {int(idx)}: ({x_adjusted}, {y})")
            
            # Initialize tracking for this point
            # self.add_tracking_point(int(idx), x_adjusted, y)
            acquired = self.points_queue_lock.acquire(timeout=1.0)
            if not acquired:
                self.get_logger().warning("Failed to acquire points queue lock in point callback")
                return
            else:
                self.add_points_queue.append((int(idx), x_adjusted, y))
                self.points_queue_lock.release()
        self.get_logger().info(f"Queued {len(data)} tracking points for addition.")
        if not self._tracking_active:
            self._tracking_active = True
            # ~30 Hz; adjust if you want lighter CPU load (e.g., 0.05 for 20 Hz)
            self.tracking_timer = self.create_timer(10.0, self.tracking_step, callback_group=self.cb_group)
            self.get_logger().info("Started tracking timer")

    
    def add_tracking_point_from_queue(self):
        """Process points in the queue to add them to tracking."""
        while self.add_points_queue:
            if self.points_queue_lock.acquire():
                point = self.add_points_queue.pop(0)
                self.points_queue_lock.release()
                idx, x, y = point
                self.add_tracking_point(idx, x, y)
            else:
                time.sleep(0.1)  # Avoid busy waiting
        self.get_logger().info("Finished processing points from queue.")
            
    def add_tracking_point(self, point_idx, x, y):
        """Add a new point to track"""
        if self.rgb_frame is None:
            self.get_logger().warning("No frame available for tracking point initialization")
            return
            
        # Find available slot
        available_slot = None
        for i in range(NUM_POINTS):
            if not self.have_point[i]:
                available_slot = i
                break
                
        if available_slot is None:
            self.get_logger().warning("No available slots for new tracking point")
            return
            
        # Initialize query features if this is the first point
        if self.query_features is None:
            frame_tensor = torch.tensor(self.rgb_frame).to(self.device)
            # Initialize with dummy points for all slots
            dummy_points = torch.zeros([NUM_POINTS, 3], dtype=torch.float32).to(self.device)
            self.query_features = self.online_model_init(
                frames=frame_tensor[None, None], 
                points=dummy_points[None, :]
            )
            self.causal_state = self.model.construct_initial_causal_state(
                NUM_POINTS, len(self.query_features.resolutions) - 1
            )
            # Move causal_state to the correct device
            self.causal_state = tree.map_structure(
                lambda x: x.to(self.device) if hasattr(x, 'to') else x, self.causal_state
            )
            self.get_logger().info("Initialized query features and causal state")
        
        # Now update the specific point
        frame_tensor = torch.tensor(self.rgb_frame).to(self.device)
        query_point = torch.tensor([0, y, x], dtype=torch.float32).to(self.device)
        
        init_query_features = self.online_model_init(
            frames=frame_tensor[None, None], 
            points=query_point[None, None]
        )
        
        # Ensure all structures are on the correct device before update
        init_query_features = tree.map_structure(
            lambda x: x.to(self.device) if hasattr(x, 'to') else x, init_query_features
        )
        self.query_features = tree.map_structure(
            lambda x: x.to(self.device) if hasattr(x, 'to') else x, self.query_features
        )
        self.causal_state = tree.map_structure(
            lambda x: x.to(self.device) if hasattr(x, 'to') else x, self.causal_state
        )
        
        # Update query features for this point
        self.query_features, self.causal_state = self.model.update_query_features(
            query_features=self.query_features,
            new_query_features=init_query_features,
            idx_to_update=np.array([available_slot]),  # numpy array as expected
            causal_state=self.causal_state,
        )
        
        self.have_point[available_slot] = True
        self.get_logger().info(f"Added tracking point {point_idx} at slot {available_slot}")
        
        # # Start tracking loop if not already running
        # if not hasattr(self, '_tracking_active'):
        #     self._tracking_active = True
        #     self.start_tracking()
        # if not self._tracking_active:
        #     self._tracking_active = True
        #     # ~30 Hz; adjust if you want lighter CPU load (e.g., 0.05 for 20 Hz)
        #     self.tracking_timer = self.create_timer(2.0, self.tracking_step, callback_group=self.cb_group)
        #     self.get_logger().info("Started tracking timer")
    
    def tracking_step(self):
        """One non-blocking tracking step driven by a ROS timer (asynchronous)."""
        if not self.step_lock.acquire(blocking=False):
            self.get_logger().debug("Skipping tracking step to avoid overlap")
            return
        
        self.get_logger().info("adding tracking points")
        self.add_tracking_point_from_queue()
        
        if not any(self.have_point):
            return
        

        acquired = self.frame_lock.acquire(timeout=1.0)
        if not acquired:
            self.get_logger().warning("Failed to acquire frame lock in tracking step")
            return
        else:
            if self.rgb_frame is None:
                return
            frame_np = self.rgb_frame.copy()
            self.frame_lock.release()

        self.get_logger().info("Performing tracking step")
        

        frame_disp = frame_np.copy()  # for imshow (optional)
        with torch.no_grad():
            if self.query_features is not None and self.causal_state is not None:
                frame_tensor = torch.from_numpy(frame_np).contiguous().to(self.device)
                track, visible, self.causal_state = self.online_model_predict(
                    frames=frame_tensor[None, None],
                    features=self.query_features,
                    causal_context=self.causal_state,
                )
                track = track.cpu().numpy()
                visible = visible.cpu().numpy()

                tracked_points = []
                for i in range(NUM_POINTS):
                    if self.have_point[i] and visible[0, i, 0]:
                        x, y = int(track[0, i, 0, 0]), int(track[0, i, 0, 1])
                        x_original = int(x + (1280 - 720) / 2)  # keep your original offset
                        pt_id = int(self.point_idx[i] if i < len(self.point_idx) else i)
                        tracked_points.append((pt_id, x_original, int(y)))
                        cv2.circle(frame_disp, (x, y), 5, (255, 0, 0), -1)
                        # save frame_disp
                        # make sure directory exists
                        if self.debug:
                            if not os.path.exists("debug_frames"):
                                os.makedirs("debug_frames")
                        cv2.imwrite(f"debug_frames/debug_tracked_frame{self.get_clock().now()}.png", frame_disp)

                if tracked_points:
                    msg_to_send = Int32MultiArray()
                    msg_to_send.data = [int(v) for triplet in tracked_points for v in triplet]
                    self.tracking_points_pub.publish(msg_to_send)
                self.get_logger().info(f"Published tracked points: {tracked_points}")
        self.step_lock.release()
        # # Non-blocking UI
        # cv2.imshow("Point Tracking", frame_disp)
        # cv2.waitKey(1)

            
    def start_tracking(self):
        """Start the main tracking loop"""
        self.get_logger().info("Starting tracking loop")
        cv2.namedWindow("Point Tracking")
        
        with torch.no_grad():
            while rclpy.ok() and any(self.have_point):
                if self.rgb_frame is None:
                    time.sleep(0.01)
                    continue
                    
                frame = self.rgb_frame.copy()
                
                if self.query_features is not None and self.causal_state is not None:
                    frame_tensor = torch.tensor(self.rgb_frame).to(self.device)
                    track, visible, self.causal_state = self.online_model_predict(
                        frames=frame_tensor[None, None],
                        features=self.query_features,
                        causal_context=self.causal_state,
                    )
                    track = track.cpu().numpy()
                    visible = visible.cpu().numpy()
                    
                    tracked_points = []
                    for i in range(NUM_POINTS):
                        if self.have_point[i] and visible[0, i, 0]:
                            x, y = int(track[0, i, 0, 0]), int(track[0, i, 0, 1])
                            # Adjust x coordinate back for publishing
                            x_original = int(x + (1280 - 720) / 2)
                            tracked_points.append((int(self.point_idx[i] if i < len(self.point_idx) else i), x_original, int(y)))
                            cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)

                    # Publish tracked points
                    if tracked_points:
                        msg_to_send = Int32MultiArray()
                        msg_to_send.data = [int(item) for sublist in tracked_points for item in sublist]
                        self.tracking_points_pub.publish(msg_to_send)

                cv2.imshow("Point Tracking", frame)
                key = cv2.waitKey(1)
                if key == 27:  # exit on ESC
                    break
                    
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    """Main function to run the point tracker node"""
    print("Welcome to the TAPIR PyTorch live demo.")
    print("Please note that if the framerate is low (<~12 fps), TAPIR performance")
    print("may degrade and you may need a more powerful GPU.")
    
    rclpy.init(args=args)
    
    try:
        # Check for debug flag in command line arguments
        import sys
        debug_mode = '--debug' in sys.argv
        
        node = PointTrackerNode(debug=True)
        # multi-threaded
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)

        # Wait for first frame and tracking points
        node.get_logger().info("Waiting for tracking point messages...")
        executor.spin()
        
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