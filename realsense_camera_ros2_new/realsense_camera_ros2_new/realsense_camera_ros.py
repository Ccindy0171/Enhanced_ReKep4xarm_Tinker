import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.ndimage import label
from PIL import Image as PILImage

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.K = np.array([
            [908.94415283, 0, 641.31561279],
            [0, 908.80529785, 370.88174438],
            [0, 0, 1]
        ])
        self.D = np.array([0, 0, 0, 0, 0])
        try:
            transform_matrix = np.load("camera_extrinsic1.npy", allow_pickle=True)
            if transform_matrix.shape == (4, 4):
                self.R = transform_matrix[:3, :3]
                self.t = transform_matrix[:3, 3:4] * 1000.0
                self.transform_matrix = transform_matrix
                self.loaded_extrinsics = True
            else:
                raise ValueError("Expected a 4x4 transformation matrix")
        except Exception as e:
            print(f"Failed to load extrinsics: {e}")
            self.R, self.t = np.eye(3), np.array([[0], [0], [0]])
            self.transform_matrix = np.eye(4)
            self.loaded_extrinsics = False
        self.rgb_sub = self.create_subscription(ROSImage, "/camera/color/image_raw", self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(ROSImage, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10)
        self.get_logger().info("RealSenseCameraNode initialized (ROS2 migration)")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

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
        c = np.uint8([[color]])
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        hue = hsvC[0][0][0]
        if hue >= 165:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:
            lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        return lowerLimit, upperLimit

    def detect_end_effector(self):
        def keep_largest_blob(image):
            binary_image = (image == 255).astype(int)
            labeled_image, num_features = label(binary_image)
            if num_features == 0:
                return np.zeros_like(image, dtype=np.uint8)
            largest_blob_label = max(range(1, num_features + 1), key=lambda lbl: np.sum(labeled_image == lbl))
            output_image = (labeled_image == largest_blob_label).astype(np.uint8) * 255
            return output_image
        color = [158, 105, 16]
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
        shape = depth_pc.shape
        xv, yv = np.meshgrid(np.arange(shape[1]), np.arange(shape[0]))
        nan_mask = ~np.isnan(depth_pc)
        xv, yv = xv[nan_mask], yv[nan_mask]
        pc_all = np.vstack((xv, yv, np.ones(xv.shape)))
        s = depth_pc[yv, xv]
        pc_camera = s * (K_inv @ pc_all)
        pw_final = (R_inv @ (pc_camera - self.t)).T
        pw_final = pw_final.reshape(shape[0], shape[1], 3)
        return pw_final

    def get_average_depth(self, x, y):
        depth_image = self.capture_points()
        neighborhood = [(dx, dy) for dx in range(-1, 2) for dy in range(-1, 2)]
        valid_depths = []
        for dx, dy in neighborhood:
            nx, ny = x + dx, y + dy
            if 0 <= nx < depth_image.shape[1] and 0 <= ny < depth_image.shape[0]:
                depth_value = depth_image[ny, nx]
                if depth_value > 0 and not np.isnan(depth_value):
                    valid_depths.append(depth_value)
        if valid_depths:
            return np.mean(valid_depths)
        else:
            print(f"Invalid depth value at ({x}, {y}): {depth_value}")

    def get_camera_coordinates(self, x, y):
        depth_value = self.get_average_depth(x, y)
        if depth_value is None or depth_value <= 0 or np.isnan(depth_value):
            print(f"Invalid depth value at ({x}, {y}): {depth_value}")
            camera_coordinates = np.array([0, 0, 0])
        else:
            camera_coordinates = (np.linalg.inv(self.K) @ np.array([x, y, 1]) * depth_value).reshape(3, 1)
        return camera_coordinates

    def get_world_coordinates(self, x, y):
        camera_coordinates = self.get_camera_coordinates(x, y)
        if camera_coordinates[0] == 0 and camera_coordinates[1] == 0 and camera_coordinates[2] == 0:
            print(f"Invalid camera coordinates at ({x}, {y}): {camera_coordinates}")
            world_coordinates = np.array([0, 0, 0])
        else:
            world_coordinates = np.linalg.inv(self.R) @ (camera_coordinates - self.t)
        return world_coordinates.flatten()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                rgb_image = node.capture_image("rgb")
                depth_image = node.capture_image("depth")
                if rgb_image is not None:
                    cv2.imshow("RGB Image", rgb_image)
                if depth_image is not None:
                    cv2.imshow("Depth Image", depth_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
