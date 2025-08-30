import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from torchvision.transforms.functional import to_tensor
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
# import pyrealsense2 as rs  # Not needed for ROS2 node, handled by RealSenseCamera
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model
from realsense_camera_ros2_new.realsense_camera_ros2_new.realsense_camera_ros import RealSenseCameraNode

class SAM:
    def __init__(self):
        sam2_checkpoint = "sam2/checkpoints/sam2.1_hiera_large.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
        sam2 = build_sam2(model_cfg, sam2_checkpoint, device=torch.device("cuda"), apply_postprocessing=False)
        self.mask_generator = SAM2AutomaticMaskGenerator(sam2)
    def generate(self, image):
        mask_dict = self.mask_generator.generate(image)
        masks = [mask_dict[i]["segmentation"] for i in range(len(mask_dict))]
        return masks

class Cutie:
    def __init__(self, init_mask, init_image):
        cutie = get_default_model()
        self.processor = InferenceCore(cutie, cfg=cutie.cfg)
        self.processor.max_internal_size = 480
        objects = np.unique(np.array(init_mask))
        objects = objects[objects != 0].tolist()
        mask = torch.from_numpy(np.array(init_mask)).cuda()
        image = to_tensor(init_image).cuda().float()
        self.processor.step(image, mask, objects=objects)
    @torch.inference_mode()
    @torch.amp.autocast('cuda')
    def generate(self, image):
        image = to_tensor(image).cuda().float()
        output_prob = self.processor.step(image)
        mask = self.processor.output_prob_to_mask(output_prob)
        mask = mask.cpu().numpy().astype(np.uint8)
        return mask

class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline_node')
        self.camera = RealSenseCameraNode()
        self.sam = SAM()
        self.mask_tracker = None
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.process)
        self.initialized = False
        self.fig, self.ax = plt.subplots()
        self.img_display = None
        plt.ion()
        plt.title("Live Camera Feed")
        plt.axis("off")

    def process(self):
        try:
            rgb = self.camera.capture_image("rgb")
            if not self.initialized:
                masks = self.sam.generate(rgb)
                self.mask_tracker = Cutie(masks, rgb)
                self.img_display = self.ax.imshow(rgb)
                self.initialized = True
            else:
                mask_track = self.mask_tracker.generate(rgb)
                self.img_display.set_data(mask_track)
                plt.draw()
                plt.pause(0.01)
        except Exception as e:
            self.get_logger().warn(f"Vision pipeline error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
