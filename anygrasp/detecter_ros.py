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
#NOTE  :  put the points in a virtual camera coordinate system to make the anygrasp output be vertical to the table
VIRTUAL_CAMERA=[[-1,0,0,0.4],
                [0,1,0,0],
                [0,0,-1,0.4],
                [0,0,0,1]]
# 配置参数
class Config:
    def __init__(self):
        self.checkpoint_path = "log/checkpoint_detection.tar"  # 模型路径
        self.max_gripper_width = 0.1
        self.gripper_height = 0.03
        self.top_down_grasp = False
        self.debug = True

cfgs = Config()

# 加载模型
anygrasp = AnyGrasp(cfgs)
anygrasp.load_net()

# RealSense 相机类
class RealSenseCamera:
    def __init__(self, node):
        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.node = node
        
        # 订阅图像话题
        self.color_sub = node.create_subscription(ROSImage, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = node.create_subscription(ROSImage, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        self.color_image = None
        self.depth_image = None
        try:
            transform_matrix = np.load("camera_extrinsic1.npy", allow_pickle=True)
            # 假设加载的是一个完整的4x4变换矩阵
            if transform_matrix.shape == (4, 4):
                # 从变换矩阵中提取旋转部分和平移部分
                self.R = transform_matrix[:3, :3]
                self.t = transform_matrix[:3, 3:4]
                self.transform_matrix = transform_matrix  # 保存完整矩阵以供需要时使用
                self.loaded_extrinsics = True
            else:
                raise ValueError("Expected a 4x4 transformation matrix")
        except Exception as e:
            print(f"Failed to load extrinsics: {e}")
            self.R, self.t = np.eye(3), np.array([[0], [0], [0]])
            self.transform_matrix = np.eye(4)
            self.loaded_extrinsics = False
        self.node.get_logger().info("RealSense camera initialized.")

    def color_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 Numpy 数组，并将 BGR 转换为 RGB
            self.color_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert color image: {e}")

    def depth_callback(self, msg):
        try:
            # 将ROS深度图像消息转换为Numpy数组
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert depth image: {e}")

    def get_images(self):
        # 返回图像数据，如果没有获取到，返回None
        if self.color_image is None or self.depth_image is None:
            self.node.get_logger().warn("Color or depth image is not available.")
            return None, None
        return self.color_image, self.depth_image
        
    def cam2virtual_camera(self, points):
        # 将点云转换到虚拟相机坐标系
        EE = VIRTUAL_CAMERA @ np.linalg.inv(self.transform_matrix)
        # 将点云转换为齐次坐标 (4xN)
        original_shape = points.shape  # 保存原始形状 (1280, 720, 3)
        points_h = np.vstack((points.reshape(-1, 3).T, np.ones((1, points.shape[0] * points.shape[1]))))  # 4xN
        # 应用变换矩阵
        transformed_points_h = EE @ points_h  # 4xN
        # 去掉齐次坐标的最后一行，并恢复原始形状
        transformed_points = transformed_points_h[:3, :].T.reshape(original_shape)  # 1280x720x3
        return transformed_points, EE
    
    def virtual_cam2world(self, gg):
        # 将虚拟相机坐标系中的抓取姿态转换到世界坐标系
        mat_anygrasp_to_ros = np.array([[0.0, 0.0, 1.0],
                                        [0.0, -1.0, 0.0],
                                        [1.0, 0.0, 0.0]])
        new_gg = []  # 创建一个新的抓取姿态列表
        for grasp in gg:
            Mat = np.linalg.inv(VIRTUAL_CAMERA)
            # 手动创建一个新的 grasp 对象
            new_grasp = GraspGroup()  # 假设 GraspGroup 是 grasp 的类
            new_grasp.translation = Mat[:3, :3] @ grasp.translation.reshape(3, 1) + Mat[:3, 3].reshape(3, 1)
            new_grasp.translation = new_grasp.translation.reshape(3,)
            new_grasp.rotation_matrix = Mat[:3, :3] @ grasp.rotation_matrix @ mat_anygrasp_to_ros
            
            new_gg.append(new_grasp)
        return new_gg

# # 初始化 RealSense 相机
# camera = RealSenseCamera()

# 找到最近的夹取姿态
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

    return closest_grasp  # 返回最接近的夹取姿态
def draw_grasp_as_coordinate_frame(grasp_pose):
    # 创建一个坐标系对象
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    # 提取抓取姿态的旋转矩阵和平移向量
    rotation_matrix = grasp_pose.rotation_matrix
    translation = grasp_pose.translation
    # mat_graspnet_to_ros = np.array([[0.0, 0.0, 1.0],
    #                                 [0.0, -1.0, 0.0],
    #                                 [1.0, 0.0, 0.0]])
    # rotation_matrix = np.dot(mat_graspnet_to_ros, np.dot(rotation_matrix, mat_graspnet_to_ros.T))
    # rotation_matrix = np.dot(rotation_matrix,mat_graspnet_to_ros)
    # 将旋转矩阵和平移向量应用到坐标系对象上
    coordinate_frame.rotate(rotation_matrix, center=(0, 0, 0))
    coordinate_frame.translate(translation)

    return coordinate_frame
# 回调函数，当接收到目标坐标点时进行推理
class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('grasp_detection_node')
        
        # Initialize camera
        self.camera = RealSenseCamera(self)
        
        # Create publisher
        self.grasp_pub = self.create_publisher(Float32MultiArray, 'grasp_pose', 10)
        
        # Create subscriber
        self.create_subscription(Point, 'target_point', self.grasp_callback, 10)
        
        self.get_logger().info('AnyGraspNode initialized')
    
    def grasp_callback(self, msg):
        target_point = np.array([msg.x, msg.y, msg.z])

        colors, depths = self.camera.get_images()
        if colors is None or depths is None:
            self.get_logger().error("Failed to get images. Skipping this callback.")
            return

        # 摄像机内参
        fx, fy = 908.9441528320312, 908.8052978515625
        cx, cy = 641.3156127929688, 370.8817443847656

        scale = 1000.0
        # set workspace to filter output grasps
        xmin, xmax = -0.2, 0.2
        ymin, ymax = -0.5, 0.5
        zmin, zmax = 0.0, 1.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        # get point cloud
        xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        # set your workspace to crop point cloud
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
        # 找到最近的夹取姿态
        closest_grasp = find_closest_grasp(gg_new, target_point)  

        if closest_grasp is not None:
            grasp_position = closest_grasp.translation
            grasp_orientation = closest_grasp.rotation_matrix
            # 返回夹取姿态到 ROS 发布者
            result_msg = Float32MultiArray()
            result_msg.data = grasp_position.tolist() + grasp_orientation.flatten().tolist()
            self.grasp_pub.publish(result_msg)
            self.get_logger().info(f"Grasp pose sent: {grasp_position}")
        # visualization

        if cfgs.debug:
            trans_mat = np.linalg.inv(VIRTUAL_CAMERA)
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            coordinate_frame = draw_grasp_as_coordinate_frame(closest_grasp)
            o3d.visualization.draw_geometries([coordinate_frame, cloud, O3D_AXIS])


def main(args=None):
    """Main function to run the grasp detection node"""
    rclpy.init(args=args)
    
    try:
        node = AnyGraspNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
