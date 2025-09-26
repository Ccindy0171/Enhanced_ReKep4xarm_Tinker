import copy
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import open3d as o3d
import os
from PIL import Image
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup, Grasp
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
        self.color_sub = node.create_subscription(ROSImage, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = node.create_subscription(ROSImage, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.received_color_image = False
        self.received_depth_image = False

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
                self.node.get_logger().info(f"Loaded extrinsics:\nRotation:\n{self.R}\nTranslation:\n{self.t.flatten()}")
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
        if not self.received_color_image:
            self.node.get_logger().info("Received first color image.")
            self.received_color_image = True

    def depth_callback(self, msg):
        try:
            # 将ROS深度图像消息转换为Numpy数组
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert depth image: {e}")
        if not self.received_depth_image:
            self.node.get_logger().info("Received first depth image.")
            self.received_depth_image = True

    def get_images(self):
        # 返回图像数据，如果没有获取到，返回None
        if self.color_image is None or self.depth_image is None:
            self.node.get_logger().warn("Color or depth image is not available.")
            return None, None
        return self.color_image, self.depth_image
        
    def cam2virtual_camera(self, points):
        # 将点云转换到虚拟相机坐标系
        EE = VIRTUAL_CAMERA @ np.linalg.inv(self.transform_matrix)
        # EE = VIRTUAL_CAMERA @ self.transform_matrix
        # 将点云转换为齐次坐标 (4xN)
        original_shape = points.shape  # 保存原始形状 (1280, 720, 3)
        points_h = np.vstack((points.reshape(-1, 3).T, np.ones((1, points.shape[0] * points.shape[1]))))  # 4xN
        # 应用变换矩阵
        transformed_points_h = EE @ points_h  # 4xN
        # 去掉齐次坐标的最后一行，并恢复原始形状
        transformed_points = transformed_points_h[:3, :].T.reshape(original_shape)  # 1280x720x3
        return transformed_points, EE
    
    def cam2world_virtual(self, points):
        # 将点云从相机坐标系转换到世界坐标系
        if not self.loaded_extrinsics:
            self.node.get_logger().warn("Extrinsics not loaded, cannot convert to world coordinates.")
            return points
        # 将点云转换为齐次坐标 (4xN)
        original_shape = points.shape  # 保存原始形状 (N, 3)
        points_h = np.vstack((points.reshape(-1, 3).T, np.ones((1, points.shape[0] * points.shape[1]))))  # 4xN
        # 应用变换矩阵
        transformed_points_h = self.transform_matrix @ points_h  # 4xN
        # 去掉齐次坐标的最后一行，并恢复原始形状
        transformed_points = transformed_points_h[:3, :].T.reshape(original_shape)  # Nx3
        return transformed_points
    
    def virtual_cam2world(self, gg):
        # 将虚拟相机坐标系中的抓取姿态转换到世界坐标系
        mat_anygrasp_to_ros = np.array([[0.0, 0.0, 1.0],
                                        [0.0, -1.0, 0.0],
                                        [1.0, 0.0, 0.0]])
        new_gg = []  # 创建一个新的抓取姿态列表
        for grasp in gg:
            Mat = np.linalg.inv(VIRTUAL_CAMERA)
            # Mat = self.transform_matrix
            # 手动创建一个新的 grasp 对象
            new_grasp = GraspGroup()  # 假设 GraspGroup 是 grasp 的类
            new_grasp.translation = Mat[:3, :3] @ grasp.translation.reshape(3, 1) + Mat[:3, 3].reshape(3, 1)
            new_grasp.translation = new_grasp.translation.reshape(3,)
            new_grasp.rotation_matrix = Mat[:3, :3] @ grasp.rotation_matrix @ mat_anygrasp_to_ros
            
            new_gg.append(new_grasp)
        return new_gg
    
    def cam2world(self, gg):
        print(gg.__class__, gg.__len__(), gg.__repr__)
        self.node.get_logger().info(f"Transforming grasps from camera to world coordinates using {self.R} and {self.t}.")
        # use self.R and self.t to convert grasp from camera to world
        mat_anygrasp_to_ros = np.array([[0.0, 0.0, 1.0],
                                        [0.0, -1.0, 0.0],
                                        [1.0, 0.0, 0.0]])
        new_gg = GraspGroup()  # 创建一个新的抓取姿态列表
        for grasp in gg:
            # 手动创建一个新的 grasp 对象
            new_grasp = copy.deepcopy(grasp)
            translation = self.R @ grasp.translation.reshape(3,1) + self.t.reshape(3,1)
            # new_grasp.translation = self.R @ grasp.translation.reshape(3,1) + self.t.reshape(3,1)
            new_grasp.translation = translation.reshape(3,)
            # print(f"Old translation: {grasp.translation}, New translation: {new_grasp.translation.flatten()}")
            new_grasp.rotation_matrix = self.R @ grasp.rotation_matrix @ mat_anygrasp_to_ros
            new_gg.add(new_grasp)
        return new_gg

    def world2cam(self, gg):
        self.node.get_logger().info(f"Transforming grasps from world to camera coordinates using {self.R.T} and {-self.R.T @ self.t}.")
        # use self.R and self.t to convert grasp from world to camera
        mat_ros_to_anygrasp = np.array([[0.0, 0.0, 1.0],
                                        [0.0, -1.0, 0.0],
                                        [1.0, 0.0, 0.0]])
        new_gg = GraspGroup()  # 创建一个新的抓取姿态列表
        for grasp in gg:
            # 手动创建一个新的 grasp 对象
            new_grasp = copy.deepcopy(grasp)
            translation = self.R.T @ (grasp.translation.reshape(3,1) - self.t)
            new_grasp.translation = translation.reshape(3,)
            new_grasp.rotation_matrix = self.R.T @ grasp.rotation_matrix @ mat_ros_to_anygrasp
            
            new_gg.add(new_grasp)
        return new_gg
    
    def world2cam_coordinates(self, world_coordinates):
        cam_coordinates = self.R.T @ (world_coordinates.reshape(3,1) - self.t)
        return cam_coordinates.reshape(3,)
    
    def world_to_pixel_coordinates(self, world_coordinates):
        """
        Project a 3D point in base/world frame into pixel coordinates of the color optical frame.

        Args:
            world_coordinates : array-like, shape (3,), meters in base/world frame.

        Uses:
            self.R : (3,3) rotation of base <- optical  (optical -> base)
            self.t : (3,)   translation of base <- optical (meters)
            self.K : (3,3) camera intrinsics for the color optical frame

        Returns:
            (u, v, Z_opt) where u,v are pixel coordinates (floats), Z_opt is depth in meters
            Returns (None, None, Z_opt) if point is behind the camera (Z_opt <= 0) or invalid.
        """
        # Ensure shapes
        fx, fy = 910.11865234, 910.26733398
        cx, cy = 648.41540527, 353.25216675

        R_bo = np.asarray(self.R, dtype=np.float64)           # base <- optical
        t_bo = np.asarray(self.t, dtype=np.float64).reshape(3, 1)
        K     = np.asarray([[fx, 0, cx],
                            [0, fy, cy],
                            [0,  0,  1]], dtype=np.float64)

        p_b = np.asarray(world_coordinates, dtype=np.float64).reshape(3, 1)

        # Invert extrinsics analytically to get optical <- base
        # R_ob = R_bo^T ; t_ob = -R_bo^T @ t_bo
        R_ob = R_bo.T
        t_ob = -R_ob @ t_bo

        # Transform base -> optical
        p_opt = R_ob @ p_b + t_ob   # (3,1)
        X, Y, Z = p_opt.flatten()

        # Point behind the camera or invalid
        if not np.isfinite(Z) or Z <= 0:
            self.node.get_logger().warn(f"Point {world_coordinates} is behind the camera or invalid (Z={Z})")
            return None

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        u = (fx * X / Z) + cx
        v = (fy * Y / Z) + cy

        # Optional: sanity check for NaNs/Infs
        if not (np.isfinite(u) and np.isfinite(v)):
            return None, None, Z
        
        self.node.get_logger().info(f"Projected world point {world_coordinates} to pixel ({u}, {v}) with depth {Z}m")

        return [round(u), round(v)]

import numpy as np

def _rpy_deg_to_rot(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic ZYX (yaw->pitch->roll). Inputs in degrees."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cy, sy = np.cos(y), np.sin(y)
    cp, sp = np.cos(p), np.sin(p)
    cr, sr = np.cos(r), np.sin(r)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]])
    return Rz @ Ry @ Rx

def _rotation_axis_angle_error(R_des, R):
    """
    Smallest angle (rad) to rotate R onto R_des via R_err = R_des^T * R.
    Robust to 0/π edge cases.
    """
    R_err = R_des.T @ R
    # Clamp numerical noise
    trace_val = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(trace_val)

def _rpy_from_R(R):
    """
    Extract yaw (Z) and pitch (Y) from intrinsic ZYX convention.
    Returns (yaw_rad, pitch_rad). We don't need roll for the upright/front-facing bias.
    """
    # For ZYX: yaw = atan2(R21, R11); pitch = asin(-R31)
    yaw = np.arctan2(R[1,0], R[0,0])
    pitch = np.arcsin(-np.clip(R[2,0], -1.0, 1.0))
    roll = np.arctan2(R[2,1], R[2,2])
    return yaw, pitch, roll

def find_closest_grasp_weighted(
    gg,
    target_point,
    desired_rpy_deg=(180.0, 0.0, 0.0),
    pos_w=1.0,
    ang_w=0.15,
    upright_w=0.0,
    front_w=0.0,
    rot_w=0.0,
    pitch_cap_deg=45.0,
    yaw_cap_deg=45.0,
    roll_cap_deg=90.0
):
    """
    Args
    ----
    gg : iterable of grasp objects with `.translation` (3,) and `.rotation_matrix` (3,3).
    target_point : np.ndarray shape (3,)
    desired_rpy_deg : tuple (roll, pitch, yaw) in degrees you want to bias toward.
                      Default = (180, 0, 0) i.e., upside-down roll, upright & facing forward.
    pos_w : weight for Euclidean distance to target (meters).
    ang_w : weight for general orientation error (radians) to desired RPY.
    upright_w : extra weight on |pitch| (radians) to keep upright (pitch≈0).
    front_w : extra weight on |yaw| (radians) to keep front-facing (yaw≈0).
    pitch_cap_deg, yaw_cap_deg : cap penalties beyond these angles so outliers don't dominate.

    Returns
    -------
    best_grasp, best_index, best_score
    """

    # for grasp in list(gg):
    #     R = np.asarray(grasp.rotation_matrix).reshape(3,3)
    #     R_yaw_180 = np.array([[-1, 0, 0],
    #                           [ 0,-1, 0],
    #                           [ 0, 0, 1]])
    #     new_grasp = copy.deepcopy(grasp)
    #     new_grasp.translation = grasp.translation
    #     new_grasp.rotation_matrix = R @ R_yaw_180
    #     gg.add(new_grasp)

    R_des = _rpy_deg_to_rot(*desired_rpy_deg)

    best_score = float('inf')
    best_grasp = None
    best_idx = -1

    pitch_cap = np.deg2rad(pitch_cap_deg)
    yaw_cap = np.deg2rad(yaw_cap_deg)
    roll_cap = np.deg2rad(roll_cap_deg)

    for i, grasp in enumerate(gg):
        p = np.asarray(grasp.translation).reshape(3)
        R = np.asarray(grasp.rotation_matrix).reshape(3,3)

        # 1) Position cost
        pos_cost = np.linalg.norm(p - target_point)

        # 2) General orientation distance (axis-angle to desired)
        ang_cost = _rotation_axis_angle_error(R_des, R)

        # 3) Upright (pitch≈0) & front-facing (yaw≈0) specific penalties
        yaw, pitch, roll = _rpy_from_R(R)
        # Soft-cap to avoid overshooting on bad grasps
        pitch_pen = min(abs(pitch), pitch_cap)
        yaw_pen = min(abs(yaw), yaw_cap)
        roll_pen = min(abs(roll), roll_cap)

        score = pos_w * pos_cost + ang_w * ang_cost + upright_w * pitch_pen + front_w * yaw_pen + rot_w * roll_pen

        # Tie-breakers: prefer smaller pos_cost first, then smaller ang_cost
        if (score < best_score or
            (np.isclose(score, best_score) and pos_cost < np.linalg.norm(best_grasp.translation - target_point)) or
            (np.isclose(score, best_score) and np.isclose(pos_cost, np.linalg.norm(best_grasp.translation - target_point))
             and ang_cost < _rotation_axis_angle_error(R_des, best_grasp.rotation_matrix))):
            best_score = score
            best_grasp = grasp
            best_idx = i

    if best_grasp is not None:
        print(f"[Grasp Selection] index={best_idx}")
        print(f"  position: {np.array(best_grasp.translation)}")
        print(f"  R:\n{np.array(best_grasp.rotation_matrix)}")
        print(f"  combined score: {best_score:.4f}")

    return best_grasp, best_idx, gg

# 找到最近的夹取姿态
def find_closest_grasp(gg, target_point):

    min_distance = float('inf')
    closest_grasp = None
    closest_grasp_idx = -1

    for i in range(len(gg)):
        grasp = gg[i]
        grasp_position = grasp.translation
        grasp_rotation = grasp.rotation_matrix
        distance = np.linalg.norm(grasp_position - target_point)

        if distance < min_distance:
            min_distance = distance
            closest_grasp = grasp
            closest_grasp_idx = i

    if closest_grasp is not None:
        print(f"Closest Grasp position: {closest_grasp.translation}")
        print(f"Grasp rotation matrix: {closest_grasp.rotation_matrix}")
        print(f"Distance to target: {min_distance}")

    return closest_grasp, closest_grasp_idx  # 返回最接近的夹取姿态
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
        self.grasp_pub = self.create_publisher(Float32MultiArray, '/grasp_pose', 10)
        
        # Create subscriber
        self.create_subscription(Point, '/target_point', self.grasp_callback, 10)
        
        self.get_logger().info('AnyGraspNode initialized')

        # call grasp callback once to warm up
        self._timer_grasp = self.create_timer(3.0, self.timer_grasp_callback)
    
    def timer_grasp_callback(self):
        self.grasp_callback(Point(x=0.42503654, y=0.078, z=-0.02686956))
        self.get_logger().info('Warming up done, cancelling timer.')
        self.destroy_timer(self._timer_grasp)
    
    '''    def grasp_callback(self, msg):
        target_point = np.array([msg.x, msg.y, msg.z])
        target_pixel = self.camera.world_to_pixel_coordinates(target_point)
        self.get_logger().info(f"Received target point: {target_point}")

        colors, depths = self.camera.get_images()
        while colors is None or depths is None:
            self.get_logger().warn("Failed to get images. Waiting...")
        
        # save images for debug
        if cfgs.debug:
            if not os.path.exists('debug'):
                os.makedirs('debug')
            color_img = Image.fromarray(colors)
            depth_img = Image.fromarray(depths)
            color_img.save('debug/color.png')
            depth_img.save('debug/depth.png')

        # 摄像机内参
        fx, fy = 910.11865234, 910.26733398
        cx, cy = 648.41540527, 353.25216675

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

        mask = np.zeros(points_z.shape, dtype=bool)
        if target_pixel is not None:
            u, v = target_pixel
            self.get_logger().info(f"Target pixel coordinates: (u={u}, v={v})")
            if 0 <= u < points_z.shape[1] and 0 <= v < points_z.shape[0]:
                # Define a square region around the target pixel
                region_size_y = 200
                region_size_x = 250
                u_min = max(u - region_size_x, 0)
                u_max = min(u + region_size_x, points_z.shape[1] - 1)
                v_min = max(v - region_size_y+50, 0)
                v_max = min(v + region_size_y, points_z.shape[0] - 1)
                mask[v_min:v_max, u_min:u_max] = True
                self.get_logger().info(f"Using mask region: u[{u_min}:{u_max}], v[{v_min}:{v_max}]")
            else:
                self.get_logger().warn(f"Target pixel {target_pixel} is out of image bounds.")

        points = np.stack([points_x, points_y, points_z], axis=-1)
        # points, EE = self.camera.cam2virtual_camera(points)
        points = points[mask].astype(np.float32)
        colors = (colors[mask].astype(np.float32)) / 255.0
        print(points.min(axis=0), points.max(axis=0))

        self.get_logger().info("finding grasps...")

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            # self.get_logger().info("Anygrasp device",anygrasp.net.device)
            anygrasp.net = anygrasp.net.cuda()
        gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)
        # Move anygrasp model off GPU after inference
        if torch.cuda.is_available():
            # input("Press Enter to move model to CPU...")
            anygrasp.net = anygrasp.net.cpu()
            torch.cuda.empty_cache()

        if gg is None or len(gg) == 0:
            print('No Grasp detected after collision detection!')

        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:20]
        # print(gg_pick.scores)
        print('grasp score:', gg_pick[0].score)

        # gg_new = self.camera.virtual_cam2world(gg)
        gg_new = self.camera.cam2world(gg)
        # 找到最近的夹取姿态
        closest_grasp, closest_grasp_idx, gg_new = find_closest_grasp_weighted(gg_new, target_point)
        # closest_grasp, closest_grasp_idx = find_closest_grasp(gg_new, target_point)
        print("closest grasp:")
        print(closest_grasp)
        print('closest_grasp_idx:', closest_grasp_idx)
        print('----------end closest grasp')

        gg_old = self.camera.world2cam(gg_new)

        if closest_grasp is not None:
            grasp_position = closest_grasp.translation
            grasp_orientation = closest_grasp.rotation_matrix
            # 返回夹取姿态到 ROS 发布者
            result_msg = Float32MultiArray()
            result_msg.data = grasp_position.tolist() + grasp_orientation.flatten().tolist()
            self.grasp_pub.publish(result_msg)
            self.get_logger().info(f"Grasp pose sent: {grasp_position}")
            self.get_logger().info(f"Grasp orientation sent: {grasp_orientation}, in ypr: {np.rad2deg(_rpy_from_R(grasp_orientation))}")
        # visualization

        if cfgs.debug:
            # trans_mat = np.linalg.inv(VIRTUAL_CAMERA)
            trans_mat = np.linalg.inv(np.eye(4))
            cloud.transform(trans_mat)
            grippers = gg_old.to_open3d_geometry_list()
            grippers = gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            
            # # visualize all potential grasp positions (gg)
            # o3d.visualization.draw_geometries([*grippers, cloud])
            # o3d.visualization.draw_geometries([grippers[0], cloud])

            # coordinate_frame = draw_grasp_as_coordinate_frame(closest_grasp)


            if closest_grasp is not None:
                # closest_gripper = closest_grasp.to_open3d_geometry()
                closest_gripper = gg_old[closest_grasp_idx].to_open3d_geometry()
                closest_gripper.transform(trans_mat)
                closest_gripper.paint_uniform_color([0.0, 1.0, 0.0])  # Green color for closest grasp
                # visualize target_point as well
                target_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
                target_sphere.translate(self.camera.world2cam_coordinates(target_point))
                target_sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red color for target point
                # draw all of grasps in gg_old
                # o3d.visualization.draw_geometries([*grippers, cloud, O3D_AXIS, target_sphere])
                # draw only closest grasp
                o3d.visualization.draw_geometries([*grippers, closest_gripper, cloud, O3D_AXIS, target_sphere])



            # closest_gripper_list = closest_grasp.to_open3d_geometry_list()
            # closest_gripper = closest_gripper_list[0] if closest_gripper_list else None
            # if closest_gripper is not None:
            #     closest_gripper.transform(trans_mat)
            #     o3d.visualization.draw_geometries([closest_gripper, cloud, O3D_AXIS])
            # else:
            #     o3d.visualization.draw_geometries([cloud, O3D_AXIS])
            # o3d.visualization.draw_geometries([coordinate_frame, cloud, O3D_AXIS])
    '''
    def grasp_callback(self, msg):
        target_point = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"Received target point: {target_point}")
        target_point[2] += 0.10

        colors, depths = self.camera.get_images()
        while colors is None or depths is None:
            self.get_logger().warn("Failed to get images. Waiting...")

        # save images for debug
        if cfgs.debug:
            if not os.path.exists('debug'):
                os.makedirs('debug')
            color_img = Image.fromarray(colors)
            depth_img = Image.fromarray(depths)
            color_img.save('debug/color.png')
            depth_img.save('debug/depth.png')

        # 摄像机内参
        fx, fy = 910.11865234, 910.26733398
        cx, cy = 648.41540527, 353.25216675

        scale = 1000.0
        # 将目标点从世界坐标系转换到相机坐标系
        target_point_cam = self.camera.world2cam_coordinates(target_point)
        self.get_logger().info(f"Target point in camera coordinates: {target_point_cam}")

        # 定义抓取范围
        xmin, xmax = target_point_cam[0] - 0.1, target_point_cam[0] + 0.1
        ymin, ymax = target_point_cam[1] - 0.1, target_point_cam[1] + 0.1
        zmin, zmax = target_point_cam[2] - 0.1, target_point_cam[2] + 0.1
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        # 获取点云
        xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points.reshape(-1, 3).astype(np.float32)
        colors = (colors.reshape(-1, 3).astype(np.float32)) / 255.0

        self.get_logger().info("finding grasps...")

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            anygrasp.net = anygrasp.net.cuda()

        gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

        if torch.cuda.is_available():
            anygrasp.net = anygrasp.net.cpu()
            torch.cuda.empty_cache()

        if gg is None or len(gg) == 0:
            self.get_logger().info('No Grasp detected after collision detection!')

        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:20]
        print('grasp score:', gg_pick[0].score)

        gg_new = self.camera.cam2world(gg)
        closest_grasp, closest_grasp_idx, gg_new = find_closest_grasp_weighted(gg_new, target_point)

        if closest_grasp is not None:
            grasp_position = closest_grasp.translation
            grasp_orientation = closest_grasp.rotation_matrix
            result_msg = Float32MultiArray()
            result_msg.data = grasp_position.tolist() + grasp_orientation.flatten().tolist()
            self.grasp_pub.publish(result_msg)
            self.get_logger().info(f"Grasp pose sent: {grasp_position}")
            self.get_logger().info(f"Grasp orientation sent: {grasp_orientation}, in ypr: {np.rad2deg(_rpy_from_R(grasp_orientation))}")

        if cfgs.debug:
            trans_mat = np.linalg.inv(np.eye(4))
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)

            if closest_grasp is not None:
                closest_gripper = gg[closest_grasp_idx].to_open3d_geometry()
                closest_gripper.transform(trans_mat)
                closest_gripper.paint_uniform_color([0.0, 1.0, 0.0])  # Green color for closest grasp
                target_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
                target_sphere.translate(target_point_cam)
                target_sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red color for target point
                o3d.visualization.draw_geometries([*grippers, closest_gripper, cloud, O3D_AXIS, target_sphere])

            
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