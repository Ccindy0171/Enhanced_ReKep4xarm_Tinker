
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import torch
import os
import json
import threading
import time
import cv2
import ast
from configparser import ConfigParser
# Import other dependencies as in the original main_rekep.py
# from rekep.environment import ReKepEnv
# ...

class MainRekepNode(Node):
    def _record_video(self):
        from skimage.draw import disk, line
        def add_point_to_rgb(rgb, point, color=(255, 255, 0)):
            rr, cc = disk(point, 20, shape=rgb.shape)
            rgb[rr, cc] = color
            return rgb
        def _project_keypoints_to_img(rgb):
            projected = rgb.copy()
            for idx in self.env._keypoint_registry:
                kr = self.env._keypoint_registry[idx]
                if kr["object"] != "none":
                    pixel = kr["img_coord"]
                    displayed_text = str(idx)
                    text_length = len(displayed_text)
                    box_width = 30 + 10 * (text_length - 1)
                    box_height = 30
                    cv2.rectangle(projected, (pixel[1] - box_width // 2, pixel[0] - box_height // 2), (pixel[1] + box_width // 2, pixel[0] + box_height // 2), (255, 255, 255), -1)
                    cv2.rectangle(projected, (pixel[1] - box_width // 2, pixel[0] - box_height // 2), (pixel[1] + box_width // 2, pixel[0] + box_height // 2), (0, 0, 0), 2)
                    org = (pixel[1] - 7 * (text_length), pixel[0] + 7)
                    color = (255, 0, 0)
                    cv2.putText(projected, displayed_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            return projected
        def add_line_to_rgb(rgb, start, end, color=(0, 255, 0)):
            rr, cc = line(start[0], start[1], end[0], end[1])
            for r, c in zip(rr, cc):
                rr_disk, cc_disk = disk((r, c), radius=5)
                rgb[rr_disk, cc_disk] = color
            return rgb
        while len(self.subgoal_idxs) == 0:
            continue
        rgb = self.camera.capture_image("rgb")
        height, width, _ = rgb.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter("saved_video.mp4", fourcc, 20, (width, height))
        while not self.terminate:
            rgb = self.camera.capture_image("rgb")
            _, ee_point = self.endeffector.return_estimated_ee(self.camera, self.env.get_ee_pos())
            subgoal_point = self.env._keypoint_registry[self.subgoal_idxs[self.stage - 1]]["img_coord"]
            rgb = add_line_to_rgb(rgb, ee_point, subgoal_point, color=(0, 255, 0))
            rgb = add_point_to_rgb(rgb, ee_point, color=(255, 255, 0))
            rgb = _project_keypoints_to_img(rgb)
            out.write(rgb)
        out.release()
        print(f"Video saved at saved_video.mp4!")

    def _execute(self, rekep_program_dir):
        from std_msgs.msg import Float32MultiArray
        from rekep.utils import get_callable_grasping_cost_fn, load_functions_from_txt, print_opt_debug_dict, get_linear_interpolation_steps, spline_interpolate_poses
        import rekep.transform_utils as T
        with open(os.path.join(rekep_program_dir, 'metadata.json'), 'r') as f:
            self.program_info = json.load(f)
        self.applied_disturbance = {stage: False for stage in range(1, self.program_info['num_stages'] + 1)}
        self.env.register_keypoints(self.program_info['init_keypoint_positions'], self.camera, rekep_program_dir)
        self.constraint_fns = dict()
        for stage in range(1, self.program_info['num_stages'] + 1):
            stage_dict = dict()
            for constraint_type in ['subgoal', 'path']:
                load_path = os.path.join(rekep_program_dir, f'stage{stage}_{constraint_type}_constraints.txt')
                get_grasping_cost_fn = get_callable_grasping_cost_fn(self.env)
                stage_dict[constraint_type] = load_functions_from_txt(load_path, get_grasping_cost_fn) if os.path.exists(load_path) else []
            self.constraint_fns[stage] = stage_dict
        self.keypoint_movable_mask = np.zeros(self.program_info['num_keypoints'] + 1, dtype=bool)
        self.keypoint_movable_mask[0] = True
        self.subgoal_idxs = self._get_all_subgoals(rekep_program_dir)
        self._update_stage(1)
        while True:
            scene_keypoints = self.env.get_keypoint_positions()
            self.keypoints = np.concatenate([[self.env.get_ee_pos()], scene_keypoints], axis=0)
            self.curr_ee_pose = self.env.get_ee_pose()
            print("Current ee pose:", self.curr_ee_pose)
            self.curr_joint_pos = self.env.get_arm_joint_positions()
            print("Current joint pos:", self.curr_joint_pos)
            self.sdf_voxels = self.env.get_sdf_voxels(self.config['sdf_voxel_size'])
            self.collision_points = self.env.get_collision_points()
            print("Stage:", self.stage)
            if self.subgoal_opt and self.is_grasp_stage == False:
                next_subgoal = self._get_next_subgoal(from_scratch=self.first_iter)
                print("Next subgoal1:", next_subgoal)
            else:
                xyz = self.keypoints[self.subgoal_idxs[self.stage - 1]]
                if self.is_grasp_stage:
                    target_point = Point()
                    target_point.x = xyz[0]/1000.0
                    target_point.y = xyz[1]/1000.0
                    target_point.z = xyz[2]/1000.0
                    self.get_logger().info(f"Sending target point: ({target_point.x}, {target_point.y}, {target_point.z})")
                    self.grasp_pub.publish(target_point)
                    # ROS2 equivalent for wait_for_message
                    # TODO: Implement a proper ROS2 wait_for_message or subscription callback
                    # For now, skip grasp_msg handling
                    next_subgoal = xyz  # Placeholder
                else:
                    next_subgoal = np.concatenate([xyz, self.curr_ee_pose[3:]])
                    print("self.subgoal_idxs[self.stage - 1]", self.subgoal_idxs[self.stage - 1])
                    print("self.keypoints", self.keypoints)
                    print("Next subgoal from keypoint:", next_subgoal)
                grasp_offset = np.array([0, 0, -10])
                subgoal_pose_homo = T.convert_pose_quat2mat(next_subgoal)
                next_subgoal[:3] += subgoal_pose_homo[:3, :3] @ grasp_offset
            print("Next subgoal:", next_subgoal)
            if self.path_opt:
                next_path = self._get_next_path(next_subgoal, from_scratch=self.first_iter)
            else:
                num_points = 100
                next_path = np.zeros((num_points, 8))
                goal_lin = np.linspace(self.curr_ee_pose, next_subgoal, num=num_points)
                next_path[:, :7] = goal_lin
            self.first_iter = False
            self.action_queue = next_path.tolist()
            print("Action shape:", np.array(self.action_queue).shape)
            self.env.execute_action(self.action_queue)
            if self.is_grasp_stage:
                self._execute_grasp_action()
            elif self.is_release_stage:
                self._execute_release_action()
            if self.stage == self.program_info['num_stages']:
                self.env.sleep(2.0)
                self.terminate = True
                print("Finished!")
                return
            self._update_stage(self.stage + 1)

    def _get_next_subgoal(self, from_scratch):
        from rekep.utils import print_opt_debug_dict
        import rekep.transform_utils as T
        subgoal_constraints = self.constraint_fns[self.stage]['subgoal']
        path_constraints = self.constraint_fns[self.stage]['path']
        subgoal_pose, debug_dict = self.subgoal_solver.solve(self.curr_ee_pose, self.keypoints, self.keypoint_movable_mask, subgoal_constraints, path_constraints, self.sdf_voxels, self.collision_points, self.is_grasp_stage, self.curr_joint_pos, from_scratch=from_scratch)
        subgoal_pose_homo = T.convert_pose_quat2mat(subgoal_pose)
        print("subgoal_pose_______", subgoal_pose)
        if self.is_grasp_stage:
            subgoal_pose[:3] += subgoal_pose_homo[:3, :3] @ np.array([-self.config['grasp_depth'] / 2.0, 0, 0])
        debug_dict['stage'] = self.stage
        print_opt_debug_dict(debug_dict)
        if self.visualize:
            self.visualizer.visualize_subgoal(subgoal_pose)
        return subgoal_pose

    def _get_next_path(self, next_subgoal, from_scratch):
        from rekep.utils import print_opt_debug_dict, get_linear_interpolation_steps, spline_interpolate_poses
        path_constraints = self.constraint_fns[self.stage]['path']
        path, debug_dict = self.path_solver.solve(self.curr_ee_pose, next_subgoal, self.keypoints, self.keypoint_movable_mask, path_constraints, self.sdf_voxels, self.collision_points, self.curr_joint_pos, from_scratch=from_scratch)
        print_opt_debug_dict(debug_dict)
        processed_path = self._process_path(path)
        if self.visualize:
            self.visualizer.visualize_path(processed_path)
        return processed_path

    def _process_path(self, path):
        from rekep.utils import get_linear_interpolation_steps, spline_interpolate_poses
        full_control_points = np.concatenate([
            self.curr_ee_pose.reshape(1, -1),
            path,
        ], axis=0)
        num_steps = get_linear_interpolation_steps(full_control_points[0], full_control_points[-1], self.config['interpolate_pos_step_size'], self.config['interpolate_rot_step_size'])
        dense_path = spline_interpolate_poses(full_control_points, num_steps)
        ee_action_seq = np.zeros((dense_path.shape[0], 8))
        ee_action_seq[:, :7] = dense_path
        ee_action_seq[:, 7] = self.env.get_gripper_null_action()
        return ee_action_seq

    def _update_stage(self, stage):
        self.stage = stage
        self.is_grasp_stage = self.program_info['grasp_keypoints'][self.stage - 1] != -1
        self.is_release_stage = self.program_info['release_keypoints'][self.stage - 1] != -1
        assert self.is_grasp_stage + self.is_release_stage <= 1, "Cannot be both grasp and release stage"
        if self.is_grasp_stage:
            self.env.open_gripper()
        self.action_queue = []
        self._update_keypoint_movable_mask()
        if stage == 1:
            self.first_iter = True

    def _update_keypoint_movable_mask(self):
        for i in range(1, len(self.keypoint_movable_mask)):
            keypoint_object = self.env.get_object_by_keypoint(i - 1)
            if self.is_grasp_stage:
                self.keypoint_movable_mask[self.subgoal_idxs[self.stage - 1]+1] = True
            print(f"Keypoint {i} movable mask: {self.keypoint_movable_mask[i]}")

    def _execute_grasp_action(self):
        import rekep.transform_utils as T
        pregrasp_pose = self.env.get_ee_pose()
        print("pregrasp_pose", pregrasp_pose)
        pregrasp_pose[:3] += T.quat2mat(pregrasp_pose[3:]) @ np.array([0, 0, self.config['grasp_depth']])
        grasp_action = np.concatenate([pregrasp_pose, [self.env.get_gripper_close_action()]])
        grasp_action = grasp_action.reshape(1, -1)
        self.env.execute_action(grasp_action, precise=True)

    def _execute_release_action(self):
        self.env.open_gripper()
    def _get_all_subgoals(self, task_dir):
        with open(os.path.join(task_dir, 'output_raw.txt'), 'r') as f:
            prompt_1 = f.read()
        prompt_2 = "Without providing any explanation, return an python integer list that has the keypoint indices that the end-effector needs to be at for each stage."
        messages = [{"role": "user", "content": [{"type": "text", "text": prompt_1 + ".\n" + prompt_2}]}]
        stream = self.ai_client.chat.completions.create(model='gpt-4o', messages=messages, temperature=0.0, max_tokens=2048, stream=True)
        output = ""
        start = time.time()
        for chunk in stream:
            print(f'[{time.time()-start:.2f}s] Querying OpenAI API...', end='\r')
            if chunk.choices[0].delta.content is not None:
                output += chunk.choices[0].delta.content
        print(f'[{time.time()-start:.2f}s] Querying OpenAI API...Done')
        output = output.replace("```", "").replace("python", "")
        print(output)
        return ast.literal_eval(output)

    def _get_target_keypoints(self, task_dir):
        with open(os.path.join(task_dir, 'output_raw.txt'), 'r') as f:
            prompt_1 = f.read()
        prompt_2 = "Without providing any explanation, return an python integer list that has the all the keypoints indices that this task to use "
        messages = [{"role": "user", "content": [{"type": "text", "text": prompt_1 + ".\n" + prompt_2}]}]
        stream = self.ai_client.chat.completions.create(model='gpt-4o', messages=messages, temperature=0.0, max_tokens=2048, stream=True)
        output = ""
        start = time.time()
        for chunk in stream:
            print(f'[{time.time()-start:.2f}s] Querying OpenAI API...', end='\r')
            if chunk.choices[0].delta.content is not None:
                output += chunk.choices[0].delta.content
        print(f'[{time.time()-start:.2f}s] Querying OpenAI API...Done')
        output = output.replace("```", "").replace("python", "")
        print(output)
        return ast.literal_eval(output)

    # ... (other methods: _record_video, _execute, _get_next_subgoal, _get_next_path, _process_path, _update_stage, _update_keypoint_movable_mask, _execute_grasp_action, _execute_release_action)
    def perform_task(self, instruction, rekep_program_dir=None, disturbance_seq=None):
        # Adapted from main_rekep.py, using rclpy logging and publishers
        rgb = self.camera.capture_image("rgb")
        points = self.camera.pixel_to_3d_points()
        mask_dir = 'mask'
        mask_files = [f for f in os.listdir(mask_dir) if f.endswith('.png')]
        mask = []
        for mask_file in mask_files:
            mask_path = os.path.join(mask_dir, mask_file)
            single_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            self.get_logger().info(f"Loaded mask {mask_file} with shape: {single_mask.shape}")
            _, single_mask_bin = cv2.threshold(single_mask, 127, 255, cv2.THRESH_BINARY)
            mask.append(single_mask_bin)

        if rekep_program_dir is None:
            keypoints, pixels, projected_img = self.keypoint_proposer.get_keypoints(rgb, points, mask)
            self.get_logger().info(f'Got {len(keypoints)} proposed keypoints')
            if self.visualize:
                self.visualizer.show_img(projected_img)
            metadata = {'init_keypoint_positions': keypoints, 'num_keypoints': len(keypoints)}
            rekep_program_dir = self.constraint_generator.generate(projected_img, instruction, metadata)
            self.get_logger().info('Constraints generated')
            self.tarcking_keypoints_idx = self._get_target_keypoints(rekep_program_dir)
            self.get_logger().info(f'Got {len(self.tarcking_keypoints_idx)} target keypoints {self.tarcking_keypoints_idx}')
            tracking_points = []
            for i in self.tarcking_keypoints_idx:
                tracking_points.append([i, pixels[i][1], pixels[i][0]])
                if pixels[i][1] < 280:
                    self.get_logger().warn(f'Warning: {i} is out of range')
            self.get_logger().info(f'Got {len(tracking_points)} target keypoints {tracking_points}')
            msg = Int32MultiArray()
            msg.data = [item for sublist in tracking_points for item in (sublist[:1] + sublist[1:])]
            self.get_logger().info(f"Sending: {msg.data}")
            self.pub.publish(msg)

        # Threaded execution (as in original)
        thread1 = threading.Thread(target=self._execute, args=(rekep_program_dir,))
        thread1.start()
        thread1.join()
        self.robot.disconnect()
        self.camera.close()

    def __init__(self, scene_file, visualize=False):
        super().__init__('rekep_main')
        self.get_logger().info('MainRekepNode initialized (ROS2 migration)')

        # Load config and initialize components (adapted from main_rekep.py)
        from rekep.utils import get_config
        global_config = get_config(config_path="./rekep/configs/config.yaml")
        self.config = global_config['main']
        self.visualize = visualize
        self.tarcking_keypoints_idx = []
        np.random.seed(self.config['seed'])
        torch.manual_seed(self.config['seed'])
        torch.cuda.manual_seed(self.config['seed'])

        # Initialize publishers
        self.pub = self.create_publisher(Int32MultiArray, '/tracking_points', 10)
        self.grasp_pub = self.create_publisher(Point, 'target_point', 10)

        # Initialize robot, camera, and other modules (imports must be ROS2 compatible)
        from rekep.keypoint_proposal import KeypointProposer
        from rekep.constraint_generation import ConstraintGenerator
        from rekep.ik_solver import xArmIKSolver
        from rekep.subgoal_solver import SubgoalSolver
        from rekep.path_solver import PathSolver
        from rekep.visualizer import Visualizer
        import rekep.transform_utils as T
        from rekep.environment import ReKepEnv
        from endeffector import EndEffector
        from xarm.wrapper import XArmAPI
        from realsense_camera_ros import RealSenseCamera
        from openai import OpenAI

        parser = ConfigParser()
        parser.read('robot.conf')
        try:
            ip = parser.get('xArm', 'ip')
        except:
            ip = '192.168.1.237'

        self.keypoint_proposer = KeypointProposer(global_config['keypoint_proposer'])
        self.constraint_generator = ConstraintGenerator(global_config['constraint_generator'])
        self.endeffector = EndEffector()
        self.robot = XArmAPI(ip)
        self.robot.motion_enable(True)
        self.robot.clean_error()
        self.robot.set_mode(6)
        self.robot.set_state(0)
        self.robot.set_servo_angle(angle=[30,-40,-30,0,50,0], speed=50)
        self.robot.set_gripper_mode(0)
        self.robot.set_gripper_enable(True)
        self.robot.set_gripper_position(850, wait=True)
        self.robot.set_mode(0)
        self.robot.set_state(0)
        self.get_logger().info("xArm Connected & Gone Home!")
        time.sleep(1)

        self.camera = RealSenseCamera()
        self.endeffector.get_point_to_world_conversion(self.camera)
        self.env = ReKepEnv(global_config['env'], self.robot, self.camera, self.endeffector)
        ik_solver = xArmIKSolver(self.env.robot)
        reset_joint_pos = self.env.get_arm_joint_positions()
        self.subgoal_opt = True
        self.path_opt = False
        if self.subgoal_opt:
            self.subgoal_solver = SubgoalSolver(global_config['subgoal_solver'], ik_solver, reset_joint_pos)
        if self.path_opt:
            self.path_solver = PathSolver(global_config['path_solver'], ik_solver, reset_joint_pos)
        self.visualizer = Visualizer(global_config['visualizer'], self.env)
        self.visualize = True
        self.ai_client = OpenAI(api_key=os.environ['OPENAI_API_KEY'])
        self.terminate = False
        self.video_save = []
        self.subgoal_idxs = []
        # Additional initialization as needed

        # TODO: Port perform_task and other methods, updating rospy to rclpy usage

def main(args=None):
    rclpy.init(args=args)
    # TODO: Parse arguments as needed (see argparse usage in original main_rekep.py)
    node = MainRekepNode(scene_file='TODO', visualize=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
