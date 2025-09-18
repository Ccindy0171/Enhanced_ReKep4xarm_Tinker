import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.ndimage import label
from PIL import Image as PILImage


class RealSenseCamera(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # 创建 CvBridge 对象，用于将 ROS 图像消息转换为 OpenCV 图像
        self.bridge = CvBridge()

        # 订阅 RealSense 相机的 RGB 和深度图像
        self.rgb_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10)  # NOTE：must sub the aligned depth image
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.camera_info_callback, 10)
        self.rgb_image = None
        self.depth_image = None
        self.received_camera_info = False
        self.received_rgb_image = False
        self.received_depth_image = False

        self.K = np.array([
            [908.94415283, 0, 641.31561279],
            [0, 908.80529785, 370.88174438],
            [0, 0, 1]
        ])
        
        # 畸变系数 D
        self.D = np.array([0, 0, 0, 0, 0])  

        try:
            transform_matrix = np.load("camera_extrinsic1.npy", allow_pickle=True)
            # 假设加载的是一个完整的4x4变换矩阵
            if transform_matrix.shape == (4, 4):
                # 从变换矩阵中提取旋转部分和平移部分
                self.R = transform_matrix[:3, :3]
                self.t = transform_matrix[:3, 3:4]*1000.0  # 保持列向量形式  转换为mm
                self.transform_matrix = transform_matrix  # 保存完整矩阵以供需要时使用
                self.loaded_extrinsics = True
                self.get_logger().info(f"Loaded extrinsics:\nRotation:\n{self.R}\nTranslation:\n{self.t.flatten()}")
            else:
                raise ValueError("Expected a 4x4 transformation matrix")
        except Exception as e:
            self.get_logger().warn(f"Failed to load extrinsics: {e}")
            self.R, self.t = np.eye(3), np.array([[0], [0], [0]])
            self.transform_matrix = np.eye(4)
            self.loaded_extrinsics = False
            
        self.get_logger().info("RealSenseCamera initialized")

    def rgb_callback(self, msg):
        """处理接收到的 RGB 图像消息"""
        if not self.received_rgb_image:
            self.get_logger().info("Received RGB image")
            self.received_rgb_image = True
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        """处理接收到的深度图像消息"""
        if not self.received_depth_image:
            self.get_logger().info("Received depth image")
            self.get_logger().info(f"Depth image encoding: {msg.encoding}")
            self.get_logger().info(f"Depth image dimensions: {msg.width}x{msg.height}")
            self.received_depth_image = True
        # Use "passthrough" to preserve original data format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def camera_info_callback(self, msg):
        """处理相机内参"""
        if self.received_camera_info:
            return  # 只处理一次
        self.get_logger().info("Received camera info")
        self.received_camera_info = True
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)  # 畸变系数

    def capture_image(self, image_type):
        if image_type == "rgb":
            if self.rgb_image is None:
                raise ValueError("RGB image is not available yet!")
            return self.rgb_image
        elif image_type == "depth":
            if self.depth_image is None:
                raise ValueError("Depth image is not available yet!")
            return self.depth_image
        else:
            raise Exception("Invalid image type!")

    def hsv_limits(self, color):
        c = np.uint8([[color]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

        return lowerLimit, upperLimit

    def detect_end_effector(self):
        def keep_largest_blob(image):
            # Ensure the image contains only 0 and 255
            binary_image = (image == 255).astype(int)

            # Label connected components
            labeled_image, num_features = label(binary_image)

            # If no features, return the original image
            if num_features == 0:
                return np.zeros_like(image, dtype=np.uint8)

            # Find the largest component by its label
            largest_blob_label = max(range(1, num_features + 1), key=lambda lbl: np.sum(labeled_image == lbl))

            # Create an output image with only the largest blob
            output_image = (labeled_image == largest_blob_label).astype(np.uint8) * 255

            return output_image

        color = [158, 105, 16]

        # Get bounding box around object
        frame = self.capture_image("rgb")
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = self.hsv_limits(color=color)
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        mask = keep_largest_blob(mask)
        mask_ = PILImage.fromarray(mask)
        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        cv2.imwrite("calib.png", frame)

        return [int((x1 + x2) / 2), int((y1 + y2) / 2)], frame

    def capture_points(self):
        return self.capture_image("depth")

    def pixel_to_3d_points(self):
        depth_pc = self.capture_points()
        
        # Convert depth from millimeters to meters
        depth_pc = depth_pc.astype(float) / 1000.0
        
        # Get camera intrinsic parameters
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        
        # Get array dimensions
        H, W = depth_pc.shape
        
        # Create coordinate grids
        points_x = np.repeat(np.expand_dims(np.arange(0, W), axis=0), H, axis=0)  # x coordinates (columns)
        points_y = np.repeat(np.expand_dims(np.arange(0, H), axis=1), W, axis=1)  # y coordinates (rows)
        
        # Apply camera projection to get 3D coordinates in camera frame
        camera_x = (points_x - cx) * depth_pc / fx
        camera_y = (points_y - cy) * depth_pc / fy
        camera_z = depth_pc
        
        # Stack into point cloud
        pc_camera = np.stack([camera_x, camera_y, camera_z], axis=2)
        
        # Convert to world coordinates using extrinsic parameters
        R_inv = np.linalg.inv(self.R)
        
        # Reshape for matrix operations
        pc_camera_reshaped = pc_camera.reshape(-1, 3).T  # Shape: (3, N)
        
        # Transform to world coordinates: R_inv @ (pc_camera - t)
        pw_final_reshaped = R_inv @ (pc_camera_reshaped - self.t)
        
        # Reshape back to original image dimensions
        pw_final = pw_final_reshaped.T.reshape(H, W, 3)

        return pw_final
    
    def get_average_depth(self, x, y):
        """根据周围9个点计算深度值的平均值，并去掉无效点。"""
        depth_image = self.capture_points()

        # save depth image for debug
        cv2.imwrite("debug_depth.png", depth_image)

        
        if depth_image is None:
            print(f"Depth image is None")
            return None
            
        print(f"Depth image shape: {depth_image.shape}, dtype: {depth_image.dtype}")
        print(f"Depth image range: min={np.min(depth_image)}, max={np.max(depth_image)}")
        print(f"Checking pixel ({x}, {y}) in image of size {depth_image.shape}")
        
        # Check if the pixel is within bounds
        if not (0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]):
            print(f"Pixel ({x}, {y}) is outside image bounds {depth_image.shape}")
            return None
        
        # Check the center pixel value first
        center_depth = depth_image[y, x]
        print(f"Center pixel depth value: {center_depth}")
        
        # 定义 3x3 邻域
        neighborhood = [
            (dx, dy) for dx in range(-1, 2) for dy in range(-1, 2)
        ]
        
        valid_depths = []
        debug_info = []
        
        # 遍历 3x3 邻域并收集有效的深度值
        for dx, dy in neighborhood:
            nx, ny = x + dx, y + dy
            
            # 确保坐标在图像范围内
            if 0 <= nx < depth_image.shape[1] and 0 <= ny < depth_image.shape[0]:
                depth_value = depth_image[ny, nx]
                debug_info.append(f"({nx},{ny}): {depth_value}")
                
                # 检查深度值是否有效 (参考代码的验证逻辑)
                # RealSense深度值在毫米单位，有效范围通常是几毫米到几米
                if depth_value > 0 and not np.isnan(depth_value) and depth_value < 10000:  # < 10m in mm
                    valid_depths.append(depth_value)
        
        print(f"Neighborhood depth values: {debug_info}")
        print(f"Valid depths found: {valid_depths}")
        
        # 如果存在有效的深度值，则计算其平均值
        if valid_depths:
            avg_depth = np.mean(valid_depths)
            print(f"Average depth: {avg_depth}")
            return avg_depth
        else:
            # 如果没有有效深度值，检查是否该区域普遍没有深度数据
            print(f"No valid depth values found in the neighborhood of ({x}, {y})")
            
            # 提供一些调试信息
            region_x1 = max(0, x - 10)
            region_x2 = min(depth_image.shape[1], x + 11)
            region_y1 = max(0, y - 10)
            region_y2 = min(depth_image.shape[0], y + 11)
            
            region = depth_image[region_y1:region_y2, region_x1:region_x2]
            valid_pixels_in_region = np.count_nonzero(region)
            total_pixels_in_region = region.size
            
            print(f"In surrounding 20x20 region: {valid_pixels_in_region}/{total_pixels_in_region} pixels have valid depth")
            
            if valid_pixels_in_region == 0:
                print("This region appears to have no depth data - this could be normal for:")
                print("  - Reflective surfaces")
                print("  - Very dark or very bright objects")
                print("  - Objects too close or too far")
                print("  - Areas outside camera's depth range")
                
            return None

    def get_camera_coordinates(self, x, y):
        """根据像素坐标转换为相机坐标系中的 3D 坐标。"""
        # 获取深度图像
        depth_value = self.get_average_depth(x, y)  # 使用平均深度值
        
        # 获取深度值
        if depth_value==None or depth_value <= 0 or np.isnan(depth_value):
            # raise ValueError(f"Invalid depth value at ({x}, {y}): {depth_value}")
            print(f"Invalid depth value at ({x}, {y}): {depth_value}")
            camera_coordinates= np.array([0, 0, 0])
        else:
            # 转换深度值从毫米到米 (RealSense depth is in mm)
            depth_in_meters = depth_value / 1000.0
            
            # 获取相机内参
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            
            # 计算相机坐标系中的 3D 坐标
            # 使用正确的投影公式：X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy, Z = depth
            camera_x = (x - cx) * depth_in_meters / fx
            camera_y = (y - cy) * depth_in_meters / fy
            camera_z = depth_in_meters
            
            camera_coordinates = np.array([camera_x, camera_y, camera_z]).reshape(3, 1)
            
            self.get_logger().info(f"Depth value at ({x}, {y}): {depth_value} mm -> {depth_in_meters} m")
            self.get_logger().info(f"Camera Coordinates: [{camera_x:.4f}, {camera_y:.4f}, {camera_z:.4f}]")

        return camera_coordinates

    def get_world_coordinates(self, x, y):
        """根据像素坐标转换为世界坐标系中的 3D 坐标。"""
        # 获取相机坐标系中的 3D 坐标
        camera_coordinates = self.get_camera_coordinates(x, y)
        # print("Camera Coordinates:", camera_coordinates)
        # 使用外参矩阵进行转换
        # print("Rotation Matrix:", self.R)
        # print("Translation Vector:", self.t)
        
        # 检查相机坐标是否有效 (处理无效深度值的情况)
        if np.array_equal(camera_coordinates, np.array([0, 0, 0])) or (camera_coordinates.shape == (3, 1) and np.all(camera_coordinates == 0)):
            # raise ValueError(f"Invalid camera coordinates at ({x}, {y}): {camera_coordinates}")
            print(f"Invalid camera coordinates at ({x}, {y}): {camera_coordinates}")
            world_coordinates = np.array([0, 0, 0])
        else:
            # 确保camera_coordinates是列向量
            if camera_coordinates.shape == (3,):
                camera_coordinates = camera_coordinates.reshape(3, 1)
            
            world_coordinates = np.linalg.inv(self.R) @ (camera_coordinates - self.t)
            # alternative = self.R @ (camera_coordinates - self.t)  # 另一种计算方式
            self.get_logger().info(f"World Coordinates: {world_coordinates.flatten()}, Camera Coordinates: {camera_coordinates.flatten()}")
        return world_coordinates.flatten()  # 返回扁平化的 3D 坐标，形状为 (3,)




    def close(self):
        """Shutdown the ROS2 node"""
        self.destroy_node()


def main(args=None):
    """Main function to run the camera node"""
    rclpy.init(args=args)
    
    try:
        camera = RealSenseCamera()
        
        while rclpy.ok():
            rclpy.spin_once(camera, timeout_sec=0.1)
            
            # 获取并显示 RGB 和深度图像
            try:
                rgb_image = camera.capture_image("rgb")
                depth_image = camera.capture_image("depth")

                if rgb_image is not None:
                    cv2.imshow("RGB Image", rgb_image)
                if depth_image is not None:
                    cv2.imshow("Depth Image", depth_image)

                # 按下 'q' 键退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception:
                pass  # Images not available yet
                
    except KeyboardInterrupt:
        pass
    finally:
        # 释放资源
        if 'camera' in locals():
            camera.close()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
