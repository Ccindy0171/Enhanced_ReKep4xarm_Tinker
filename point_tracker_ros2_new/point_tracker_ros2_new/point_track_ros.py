import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import tree
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

NUM_POINTS = 8

class PointTrackerNode(Node):
    def __init__(self):
        super().__init__('point_tracking_node')
        self.bridge = CvBridge()
        self.rgb_frame = None
        self.first_frame_received = False
        self.have_point = [False] * NUM_POINTS
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = self.load_model()
        self.query_points = torch.zeros([NUM_POINTS, 3], dtype=torch.float32).to(self.device)
        self.query_features = None
        self.causal_state = None
        self.point_idx = []
        self.tracking_points_pub = self.create_publisher(Int32MultiArray, '/current_tracking_points', 10)
        self.create_subscription(ROSImage, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Int32MultiArray, '/tracking_points', self.point_callback, 10)
        self.get_logger().info('PointTrackerNode initialized (ROS2 migration)')

    def load_model(self):
        from tapnet.torch1 import tapir_model
        model = tapir_model.TAPIR(pyramid_level=1, use_casual_conv=True)
        model.load_state_dict(torch.load('checkpoints/causal_bootstapir_checkpoint.pt'))
        model = model.to(self.device)
        model = model.eval()
        torch.set_grad_enabled(False)
        return model

    def preprocess_frames(self, frames):
        frames = frames.float()
        frames = frames / 255 * 2 - 1
        return frames

    def online_model_init(self, frames, points):
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
        visibles = (1 - F.sigmoid(occlusions)) * (1 - F.sigmoid(expected_dist)) > 0.5
        return visibles

    def online_model_predict(self, frames, features, causal_context):
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
        self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.first_frame_received = True
        # Crop to square
        trunc = abs(self.rgb_frame.shape[1] - self.rgb_frame.shape[0]) // 2
        if self.rgb_frame.shape[1] > self.rgb_frame.shape[0]:
            self.rgb_frame = self.rgb_frame[:, trunc:-trunc]
        elif self.rgb_frame.shape[1] < self.rgb_frame.shape[0]:
            self.rgb_frame = self.rgb_frame[trunc:-trunc]

    def point_callback(self, msg):
        points = np.zeros((NUM_POINTS, 3), dtype=np.float32)
        data = np.array(msg.data).reshape(-1, 3)
        self.point_idx = []
        num = 0
        for point in data:
            idx, x, y = point
            self.point_idx.append(int(idx))
            x = x - (1280 - 720) / 2
            pos = (y, x)
            points[num] = np.array((0,) + pos, dtype=np.float32)
            self.have_point[num] = True
            num += 1
        self.query_points = torch.tensor(points).to(self.device)
        self.query_features = None
        self.causal_state = None
        self.track_points(num)

    def track_points(self, num):
        if self.rgb_frame is None:
            self.get_logger().warn('No RGB frame available for tracking.')
            return
        frame = torch.tensor(self.rgb_frame).to(self.device)
        if self.query_features is None:
            self.query_features = self.online_model_init(
                frames=frame[None, None], points=self.query_points[None, :]
            )
            self.causal_state = self.model.construct_initial_causal_state(
                NUM_POINTS, len(self.query_features.resolutions) - 1
            )
            self.causal_state = tree.map_structure(lambda x: x.to(self.device), self.causal_state)
        track, visible, self.causal_state = self.online_model_predict(
            frames=frame[None, None],
            features=self.query_features,
            causal_context=self.causal_state,
        )
        track = np.round(track.cpu().numpy())
        visible = visible.cpu().numpy()
        numpy_frame = self.rgb_frame.copy()
        tracked_points = []
        for i, _ in enumerate(self.have_point):
            if visible[0, i, 0] and self.have_point[i]:
                x, y = int(track[0, i, 0, 0]), int(track[0, i, 0, 1])
                tracked_points.append((int(self.point_idx[i]), int(x + (1280 - 720) / 2), int(y)))
                cv2.circle(numpy_frame, (x, y), 5, (255, 0, 0), -1)
        if tracked_points:
            msg_to_send = Int32MultiArray()
            msg_to_send.data = [item for sublist in tracked_points for item in sublist]
            self.tracking_points_pub.publish(msg_to_send)
        cv2.imshow("Point Tracking", numpy_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PointTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
