import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.ndimage import label
from PIL import Image as PILImage


class RealSenseCamera:
    def __init__(self):
        # # 初始化 ROS 节点
        # rospy.init_node("realsense_camera", anonymous=True)

        # 创建 CvBridge 对象，用于将 ROS 图像消息转换为 OpenCV 图像
        self.bridge = CvBridge()

        # 订阅 RealSense 相机的 RGB 和深度图像
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)  # NOTE：must sub the aligned depth image
        self.rgb_image = None
        self.depth_image = None

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
            else:
                raise ValueError("Expected a 4x4 transformation matrix")
        except Exception as e:
            print(f"Failed to load extrinsics: {e}")
            self.R, self.t = np.eye(3), np.array([[0], [0], [0]])
            self.transform_matrix = np.eye(4)
            self.loaded_extrinsics = False

    def rgb_callback(self, msg):
        """处理接收到的 RGB 图像消息"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        """处理接收到的深度图像消息"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")  # 深度图像是16位无符号整数

    def camera_info_callback(self, msg):
        """处理相机内参"""
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)  # 畸变系数

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
        K_inv = np.linalg.inv(self.K)
        R_inv = np.linalg.inv(self.R)

        # Get array of valid pixel locations
        shape = depth_pc.shape
        xv, yv = np.meshgrid(np.arange(shape[1]), np.arange(shape[0]))
        nan_mask = ~np.isnan(depth_pc)
        xv, yv = xv[nan_mask], yv[nan_mask]
        pc_all = np.vstack((xv, yv, np.ones(xv.shape)))

        # Convert pixel to world coordinates
        s = depth_pc[yv, xv]
        pc_camera = s * (K_inv @ pc_all)
        pw_final = (R_inv @ (pc_camera - self.t)).T
        pw_final = pw_final.reshape(shape[0], shape[1], 3)

        return pw_final
    
    def get_average_depth(self, x, y):
        """根据周围9个点计算深度值的平均值，并去掉无效点。"""
        depth_image = self.capture_points()
        
        # 定义 3x3 邻域
        neighborhood = [
            (dx, dy) for dx in range(-1, 2) for dy in range(-1, 2)
        ]
        
        valid_depths = []
        
        # 遍历 3x3 邻域并收集有效的深度值
        for dx, dy in neighborhood:
            nx, ny = x + dx, y + dy
            
            # 确保坐标在图像范围内
            if 0 <= nx < depth_image.shape[1] and 0 <= ny < depth_image.shape[0]:
                depth_value = depth_image[ny, nx]
                
                # 检查深度值是否有效
                if depth_value > 0 and not np.isnan(depth_value):
                    valid_depths.append(depth_value)
        
        # 如果存在有效的深度值，则计算其平均值
        if valid_depths:
            return np.mean(valid_depths)  # 返回平均深度值，形状是 ()
        else:
            # raise ValueError(f"Invalid depth values in the neighborhood of ({x}, {y})")
            print(f"Invalid depth value at ({x}, {y}): {depth_value}")

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
            # 计算相机坐标系中的 3D 坐标
            camera_coordinates = (np.linalg.inv(self.K) @ np.array([x, y, 1]) * depth_value).reshape(3, 1)

        return camera_coordinates

    def get_world_coordinates(self, x, y):
        """根据像素坐标转换为世界坐标系中的 3D 坐标。"""
        # 获取相机坐标系中的 3D 坐标
        camera_coordinates = self.get_camera_coordinates(x, y)
        # print("Camera Coordinates:", camera_coordinates)
        # 使用外参矩阵进行转换
        # print("Rotation Matrix:", self.R)
        # print("Translation Vector:", self.t)
        if camera_coordinates[0]==0 and camera_coordinates[1]==0 and camera_coordinates[2]==0: # 当深度值无效时全部赋值为0
            # raise ValueError(f"Invalid camera coordinates at ({x}, {y}): {camera_coordinates}")
            print(f"Invalid camera coordinates at ({x}, {y}): {camera_coordinates}")
            world_coordinates = np.array([0, 0, 0])
        else:
            world_coordinates = np.linalg.inv(self.R) @ (camera_coordinates - self.t)
            # print("World Coordinates:", world_coordinates)
        return world_coordinates.flatten()  # 返回扁平化的 3D 坐标，形状为 (3,)




    def close(self):
        rospy.signal_shutdown("Shutting down ROS node")


if __name__ == "__main__":
    # 初始化 RealSense 相机 ROS 订阅器
    camera = RealSenseCamera()

    try:
        while not rospy.is_shutdown():
            # 获取并显示 RGB 和深度图像
            rgb_image = camera.capture_image("rgb")
            depth_image = camera.capture_image("depth")

            if rgb_image is not None:
                cv2.imshow("RGB Image", rgb_image)
            if depth_image is not None:
                cv2.imshow("Depth Image", depth_image)

            # 按下 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        camera.close()
        cv2.destroyAllWindows()
