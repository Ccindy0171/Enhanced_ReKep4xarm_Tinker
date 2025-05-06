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
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NUM_POINTS = 8

# 初始化 CvBridge
bridge = CvBridge()

def preprocess_frames(frames):
  """Preprocess frames to model inputs.

  Args:
    frames: [num_frames, height, width, 3], [0, 255], np.uint8

  Returns:
    frames: [num_frames, height, width, 3], [-1, 1], np.float32
  """
  frames = frames.float()
  frames = frames / 255 * 2 - 1
  return frames


def online_model_init(frames, points):
  """Initialize query features for the query points."""
  frames = preprocess_frames(frames)
  feature_grids = model.get_feature_grids(frames, is_training=False)
  features = model.get_query_features(
      frames,
      is_training=False,
      query_points=points,
      feature_grids=feature_grids,
  )
  return features


def postprocess_occlusions(occlusions, expected_dist):
  visibles = (1 - F.sigmoid(occlusions)) * (1 - F.sigmoid(expected_dist)) > 0.5
  return visibles


def online_model_predict(frames, features, causal_context):
  """Compute point tracks and occlusions given frames and query points."""
  frames = preprocess_frames(frames)
  feature_grids = model.get_feature_grids(frames, is_training=False)
  trajectories = model.estimate_trajectories(
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
  # Take only the predictions for the final resolution.
  # For running on higher resolution, it's typically better to average across
  # resolutions.
  tracks = trajectories["tracks"][-1]
  occlusions = trajectories["occlusion"][-1]
  uncertainty = trajectories["expected_dist"][-1]
  visibles = postprocess_occlusions(occlusions, uncertainty)
  return tracks, visibles, causal_context


print("Welcome to the TAPIR PyTorch live demo.")
print("Please note that if the framerate is low (<~12 fps), TAPIR performance")
print("may degrade and you may need a more powerful GPU.")

if torch.cuda.is_available():
  device = torch.device("cuda")
else:
  device = torch.device("cpu")

# --------------------
# Load checkpoint and initialize
print("Creating model...")
model = tapir_model.TAPIR(pyramid_level=1, use_casual_conv=True)
print("Loading checkpoint...")
model.load_state_dict(
    torch.load("checkpoints/causal_bootstapir_checkpoint.pt")
)
model = model.to(device)
model = model.eval()
torch.set_grad_enabled(False)

# 初始化ROS节点
rospy.init_node('point_tracking_node')
first_frame_received = False
def callback(msg):
    """处理接收到的图像消息并转换为 NumPy 数组，裁剪为正方形。"""
    global rgb_frame,first_frame_received
    # rospy.loginfo("Received image data.")
    rgb_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    first_frame_received = True  # 标志位设置为 True
    # 图像裁剪为正方形
    trunc = np.abs(rgb_frame.shape[1] - rgb_frame.shape[0]) // 2
    if rgb_frame.shape[1] > rgb_frame.shape[0]:  # 如果宽度大于高度
        rgb_frame = rgb_frame[:, trunc:-trunc]
    elif rgb_frame.shape[1] < rgb_frame.shape[0]:  # 如果高度大于宽度
        rgb_frame = rgb_frame[trunc:-trunc]

# 订阅 RealSense 相机的图像主题
rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, lambda msg: callback(msg))

# ROS 发布器
tracking_points_pub = rospy.Publisher('/current_tracking_points', Int32MultiArray, queue_size=10)


# 存储从 ROS 接收到的图像数据
rgb_frame = None
pos = tuple()
query_frame = True
have_point = [False] * NUM_POINTS

query_points = torch.zeros([NUM_POINTS, 3], dtype=torch.float32)
query_points = query_points.to(device)
# 等待第一帧图像到达
while not first_frame_received:
    rospy.loginfo("Waiting for the first frame...")
    rospy.sleep(0.1)
frame = torch.tensor(rgb_frame).to(device)

query_features = online_model_init(
    frames=frame[None, None], points=query_points[None, :]
)

causal_state = model.construct_initial_causal_state(
    NUM_POINTS, len(query_features.resolutions) - 1
)
causal_state = tree.map_structure(lambda x: x.to(device), causal_state)

prediction, visible, causal_state = online_model_predict(
    frames=frame[None, None],
    features=query_features,
    causal_context=causal_state,
)

next_query_idx = 0

step_counter = 0



# 订阅点追踪消息
def point_callback(msg):
    num=0
    global point_idx,rgb_frame
    """处理从外部传入的追踪点信息。"""
    global query_frame,query_features,causal_state
    points= np.zeros((NUM_POINTS, 3), dtype=np.float32)
    data = np.array(msg.data).reshape(-1, 3)  # 将扁平化的列表转换为二维数组
    point_idx=[]
    for point in data:
        idx, x, y = point
        point_idx.append(int(idx))
        x = x - (1280 - 720) / 2  # 调整 x 坐标
        pos= (y, x)
        points[num] = np.array((0,) + pos, dtype=np.float32)
        rospy.loginfo(f"Received point {int(idx)}: ({x}, {y})")
        have_point[num] = True
        num+=1
    query_frame = True 

    cv2.namedWindow("Point Tracking")
    
    with torch.no_grad():
        while rgb_frame is not None:
            frame = rgb_frame
            numpy_frame = frame
            if query_frame:
                frame = torch.tensor(frame).to(device)
                for i in range(num):
                    query_points_i = torch.tensor(points[i]).to(device)
                    print(f"query_points_i shape: {query_points_i.shape}")
                    init_query_features = online_model_init(
                        frames=frame[None, None], points=query_points_i[None, None]
                    )
                    query_features, causal_state = model.update_query_features(
                        query_features=query_features,
                        new_query_features=init_query_features,
                        idx_to_update=np.array([int(i)]),  
                        causal_state=causal_state,
                    )
            query_frame = False

            if True:
                frame = torch.tensor(frame).to(device)
                track, visible, causal_state = online_model_predict(
                    frames=frame[None, None],
                    features=query_features,
                    causal_context=causal_state,
                )
                track = np.round(track.cpu().numpy())
                visible = visible.cpu().numpy()
                
                tracked_points=[]
                for i, _ in enumerate(have_point):
                    if visible[0, i, 0] and have_point[i]:
                        x, y = int(track[0, i, 0, 0]), int(track[0, i, 0, 1])
                        tracked_points.append((int(point_idx[i]), int(x + (1280 - 720) / 2), int(y)))  # 保存追踪点的索引和位置
                        cv2.circle(numpy_frame, (x, y), 5, (255, 0, 0), -1)

                # 将追踪的点位置转化为ROS消息并发布
                if tracked_points:
                    msg_to_send = Int32MultiArray()
                    msg_to_send.data = [item for sublist in tracked_points for item in sublist]  # 扁平化数据
                    tracking_points_pub.publish(msg_to_send)

            cv2.imshow("Point Tracking", numpy_frame)
            cv2.waitKey(1)


# 订阅点追踪消息       
rospy.Subscriber('/tracking_points', Int32MultiArray, point_callback)
# rospy.Subscriber("/camera/color/image_raw", Image, lambda msg: callback(msg))
# 等待触发
rospy.loginfo("Waiting for point tracking messages...")
rospy.spin()