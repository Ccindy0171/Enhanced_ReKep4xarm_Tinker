
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import open3d as o3d
import os
from PIL import Image
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2

O3D_AXIS = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=[0, 0, 0])
VIRTUAL_CAMERA = [[-1,0,0,0.4],[0,1,0,0],[0,0,-1,0.4],[0,0,0,1]]

class Config:
    def __init__(self):
        self.checkpoint_path = "log/checkpoint_detection.tar"
        self.max_gripper_width = 0.1
        self.gripper_height = 0.03
        self.top_down_grasp = False
        self.debug = True

cfgs = Config()
anygrasp = AnyGrasp(cfgs)
anygrasp.load_net()

class RealSenseCamera:
    def __init__(self, node):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.node = node
        self.color_sub = node.create_subscription(ROSImage, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = node.create_subscription(ROSImage, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        try:
            transform_matrix = np.load("camera_extrinsic1.npy", allow_pickle=True)
            if transform_matrix.shape == (4, 4):
                self.R = transform_matrix[:3, :3]
                self.t = transform_matrix[:3, 3:4]
                self.transform_matrix = transform_matrix
                self.loaded_extrinsics = True
            else:
                raise ValueError("Expected a 4x4 transformation matrix")
        except Exception as e:
            print(f"Failed to load extrinsics: {e}")
            self.R, self.t = np.eye(3), np.array([[0], [0], [0]])
            self.transform_matrix = np.eye(4)
            self.loaded_extrinsics = False
        node.get_logger().info("RealSense camera initialized.")

    def color_callback(self, msg):
        try:
            self.color_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert color image: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert depth image: {e}")

    def get_images(self):
        if self.color_image is None or self.depth_image is None:
            self.node.get_logger().warn("Color or depth image is not available.")
            return None, None
        return self.color_image, self.depth_image

    def cam2virtual_camera(self, points):
        EE = np.array(VIRTUAL_CAMERA) @ np.linalg.inv(self.transform_matrix)
        original_shape = points.shape
        points_h = np.vstack((points.reshape(-1, 3).T, np.ones((1, points.shape[0] * points.shape[1]))))
        transformed_points_h = EE @ points_h
        transformed_points = transformed_points_h[:3, :].T.reshape(original_shape)
        return transformed_points, EE

    def virtual_cam2world(self, gg):
        mat_anygrasp_to_ros = np.array([[0.0, 0.0, 1.0],[0.0, -1.0, 0.0],[1.0, 0.0, 0.0]])
        new_gg = []
        for grasp in gg:
            Mat = np.linalg.inv(np.array(VIRTUAL_CAMERA))
            new_grasp = GraspGroup()
            new_grasp.translation = Mat[:3, :3] @ grasp.translation.reshape(3, 1) + Mat[:3, 3].reshape(3, 1)
            new_grasp.translation = new_grasp.translation.reshape(3,)
            new_grasp.rotation_matrix = Mat[:3, :3] @ grasp.rotation_matrix @ mat_anygrasp_to_ros
            new_gg.append(new_grasp)
        return new_gg

def find_closest_grasp(gg, target_point):
    min_distance = float('inf')
    closest_grasp = None
    for grasp in gg:
        grasp_position = grasp.translation
        grasp_rotation = grasp.rotation_matrix
        distance = np.linalg.norm(grasp_position - target_point)
        if distance < min_distance:
            min_distance = distance
            closest_grasp = grasp
    if closest_grasp is not None:
        print(f"Closest Grasp position: {closest_grasp.translation}")
        print(f"Grasp rotation matrix: {closest_grasp.rotation_matrix}")
        print(f"Distance to target: {min_distance}")
    return closest_grasp

def draw_grasp_as_coordinate_frame(grasp_pose):
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    rotation_matrix = grasp_pose.rotation_matrix
    translation = grasp_pose.translation
    coordinate_frame.rotate(rotation_matrix, center=(0, 0, 0))
    coordinate_frame.translate(translation)
    return coordinate_frame

class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('grasp_detection_node')
        self.camera = RealSenseCamera(self)
        self.grasp_pub = self.create_publisher(Float32MultiArray, 'grasp_pose', 10)
        self.create_subscription(Point, 'target_point', self.grasp_callback, 10)
        self.get_logger().info('AnyGraspNode initialized (ROS2 migration)')

    def grasp_callback(self, msg):
        target_point = np.array([msg.x, msg.y, msg.z])
        colors, depths = self.camera.get_images()
        if colors is None or depths is None:
            self.get_logger().error("Failed to get images. Skipping this callback.")
            return
        fx, fy = 908.9441528320312, 908.8052978515625
        cx, cy = 641.3156127929688, 370.8817443847656
        scale = 1000.0
        xmin, xmax = -0.2, 0.2
        ymin, ymax = -0.5, 0.5
        zmin, zmax = 0.0, 1.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]
        xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z
        mask = (points_z > 0) & (points_z < 1)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points, EE = self.camera.cam2virtual_camera(points)
        points = points[mask].astype(np.float32)
        colors = (colors[mask].astype(np.float32)) / 255.0
        print(points.min(axis=0), points.max(axis=0))
        gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)
        if len(gg) == 0:
            print('No Grasp detected after collision detection!')
        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:20]
        print(gg_pick.scores)
        print('grasp score:', gg_pick[0].score)
        gg_new = self.camera.virtual_cam2world(gg)
        closest_grasp = find_closest_grasp(gg_new, target_point)
        if closest_grasp is not None:
            grasp_position = closest_grasp.translation
            grasp_orientation = closest_grasp.rotation_matrix
            result_msg = Float32MultiArray()
            result_msg.data = grasp_position.tolist() + grasp_orientation.flatten().tolist()
            self.grasp_pub.publish(result_msg)
            self.get_logger().info(f"Grasp pose sent: {grasp_position}")
        if cfgs.debug:
            trans_mat = np.linalg.inv(np.array(VIRTUAL_CAMERA))
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            coordinate_frame = draw_grasp_as_coordinate_frame(closest_grasp)
            o3d.visualization.draw_geometries([coordinate_frame, cloud, O3D_AXIS])

def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
