import torch
import numpy as np
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray,Float32MultiArray
from geometry_msgs.msg import Point
import argparse
from rekep.environment import ReKepEnv
from rekep.keypoint_proposal import KeypointProposer
from rekep.constraint_generation import ConstraintGenerator
from rekep.ik_solver import xArmIKSolver
from rekep.subgoal_solver import SubgoalSolver
from rekep.path_solver import PathSolver
from rekep.visualizer import Visualizer
import rekep.transform_utils as T
from rekep.utils import (
    bcolors,
    get_config,
    load_functions_from_txt,
    get_linear_interpolation_steps,
    spline_interpolate_poses,
    get_callable_grasping_cost_fn,
    print_opt_debug_dict,
)

from realsense_camera_ros import RealSenseCamera
from vision_pipeline import SAM
from xarm.wrapper import XArmAPI
from endeffector import EndEffector
from openai import OpenAI
import ast, time
import cv2
import threading
from skimage.draw import disk, line
from configparser import ConfigParser
        
        
class MainRekepNode(Node):
    def __init__(self, scene_file, visualize=False):
        super().__init__('rekep_main_node')
        
        # Read robot configuration
        parser = ConfigParser()
        parser.read('robot.conf')
        try:
            ip = parser.get('xArm', 'ip')
        except:
            ip = input('Please input the xArm ip address[192.168.1.237]:')
            if not ip:
                ip = '192.168.1.237'
        
        global_config = get_config(config_path="./rekep/configs/config.yaml")
        self.config = global_config['main']
        self.bounds_min = np.array(self.config['bounds_min'])
        self.bounds_max = np.array(self.config['bounds_max'])
        self.visualize = visualize
        self.tarcking_keypoints_idx = []
        
        # set random seed
        np.random.seed(self.config['seed'])
        torch.manual_seed(self.config['seed'])
        torch.cuda.manual_seed(self.config['seed'])
        
        # initialize keypoint proposer and constraint generator
        self.keypoint_proposer = KeypointProposer(global_config['keypoint_proposer'])
        self.constraint_generator = ConstraintGenerator(global_config['constraint_generator'])
        self.endeffector = EndEffector()
        
        # Initialize robot and camera
        self.robot = XArmAPI(ip)
        self.robot.motion_enable(True)
        self.robot.clean_error()
        self.robot.set_mode(6)
        self.robot.set_state(0)
        self.robot.set_servo_angle(angle=[30,-40,-30,0,50,0], speed=50)
        self.robot.set_gripper_mode(0)
        self.robot.set_gripper_enable(True)
        self.robot.set_gripper_position(850, wait=True)
        self.robot.set_mode(0)  # 设置为位置控制模式
        self.robot.set_state(0)
        self.get_logger().info("xArm Connected & Gone Home!")
        time.sleep(1)

        self.camera = RealSenseCamera()
        # self.sam = SAM()

        # Get point to world conversion
        self.endeffector.get_point_to_world_conversion(self.camera)

        # initialize environment (real world)
        self.env = ReKepEnv(global_config['env'], self.robot, self.camera, self.endeffector, self)

        # ik_solver
        ik_solver = xArmIKSolver(self.env.robot)

        # initialize solvers
        reset_joint_pos = self.env.get_arm_joint_positions()
        self.subgoal_opt = True   # can choose to optimize subgoal or directly go to keypoint positions
        self.path_opt = False   # can choose to optimize path or directly interpolate to save time
        if self.subgoal_opt:
            self.subgoal_solver = SubgoalSolver(global_config['subgoal_solver'], ik_solver, reset_joint_pos)
        if self.path_opt:
            self.path_solver = PathSolver(global_config['path_solver'], ik_solver, reset_joint_pos)
        
        # initialize visualizer
        self.visualizer = Visualizer(global_config['visualizer'], self.env)
        self.visualize = True
        # OpenAI client
        self.ai_client = OpenAI(api_key=os.environ['OPENAI_API_KEY'])

        self.terminate = False
        self.video_save = []
        self.subgoal_idxs = []

        # ROS2 publishers
        self.pub = self.create_publisher(Int32MultiArray, '/tracking_points', 10)
        self.grasp_pub = self.create_publisher(Point, 'target_point', 10)
        
        # Storage for received grasp message
        self.received_grasp_msg = None
        
        # Create subscription for grasp poses
        self.grasp_sub = self.create_subscription(
            Float32MultiArray, 'grasp_pose', self.grasp_callback, 10)
        
        self.get_logger().info("MainRekepNode initialized")
        
    def grasp_callback(self, msg):
        """Callback to receive grasp pose messages"""
        self.received_grasp_msg = msg
        self.get_logger().info(f"Received grasp pose: {msg.data}")
        
    def wait_for_grasp_message(self, timeout=10.0):
        """Wait for grasp message with timeout"""
        self.received_grasp_msg = None
        start_time = time.time()
        
        while self.received_grasp_msg is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.received_grasp_msg is None:
            self.get_logger().warn("Timeout occurred while waiting for grasp pose.")
            return None
        else:
            return self.received_grasp_msg

    def perform_task(self, instruction, rekep_program_dir=None, disturbance_seq=None):
        rgb = self.camera.capture_image("rgb")
        points = self.camera.pixel_to_3d_points()
        # mask = self.sam.generate(rgb)
        # save mask
        # 遍历每个掩码并保存
        # for i, single_mask in enumerate(mask):
        #     single_mask = single_mask.astype(np.uint8) * 255
        #     cv2.imwrite(f'mask_{i}.png', single_mask)

        # 从本地文件夹‘mask’读取所有掩码，数量根据文件夹中图片数量决定
        mask_dir = 'mask'  # 假设掩码文件夹是'mask'
        mask_files = [f for f in os.listdir(mask_dir) if f.endswith('.png')]  # 只读取png文件
        mask = []

        for mask_file in mask_files:
            mask_path = os.path.join(mask_dir, mask_file)
            single_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)  # 读取为灰度图
            print(f"Loaded mask {mask_file} with shape: {single_mask.shape}")
            # 确保掩码是二值化的
            _, single_mask_bin = cv2.threshold(single_mask, 127, 255, cv2.THRESH_BINARY)

            # 将二值化掩码添加到masks列表
            mask.append(single_mask_bin)

        # ====================================
        # = keypoint proposal and constraint generation
        # ====================================
        if rekep_program_dir is None:
            keypoints,pixels, projected_img = self.keypoint_proposer.get_keypoints(rgb, points, mask)
            print(f'{bcolors.HEADER}Got {len(keypoints)} proposed keypoints{bcolors.ENDC}')
            if self.visualize:
                self.visualizer.show_img(projected_img)
            metadata = {'init_keypoint_positions': keypoints, 'num_keypoints': len(keypoints)}
            # metadata = {'init_keypoint_pixels': pixels, 'num_keypoints': len(pixels)}
            rekep_program_dir = self.constraint_generator.generate(projected_img, instruction, metadata)
            print(f'{bcolors.HEADER}Constraints generated{bcolors.ENDC}')
            # ask the gpt for targets tracking keypoints
            self.tarcking_keypoints_idx = self._get_target_keypoints(rekep_program_dir)
            print(f'{bcolors.HEADER}Got {len(self.tarcking_keypoints_idx)} target keypoints{self.tarcking_keypoints_idx}{bcolors.ENDC}')
            # 使用tarcking_keypoints_idx筛选出要保留的pixels
            tracking_points = []
            for i in self.tarcking_keypoints_idx:
                tracking_points.append([i, pixels[i][1], pixels[i][0]])
                if pixels[i][1]<280:
                    # warning
                    print(f'{bcolors.WARNING}Warning: {i} is out of range{bcolors.ENDC}')
            print(f'{bcolors.HEADER}Got {len(tracking_points)} target keypoints{tracking_points}{bcolors.ENDC}')

            # 使用ros 发布器发布
            msg = Int32MultiArray()
            # 展平为1维整数数组
            msg.data = [item for sublist in tracking_points for item in (sublist[:1] + sublist[1:])]
            self.get_logger().info(f"Sending: {msg.data}")
            self.pub.publish(msg)

        # ====================================
        # = execute
        # ====================================
        # Create threads
        thread1 = threading.Thread(target=self._execute, args=(rekep_program_dir,))
        # thread2 = threading.Thread(target=self._record_video)

        # # Start threads
        thread1.start()
        # thread2.start()

        # # Wait until both threads complete
        thread1.join()
        # thread2.join()

        # Cleanup
        self.robot.disconnect()
        self.camera.close()


    def _get_all_subgoals(self, task_dir):

        # save prompt
        with open(os.path.join(task_dir, 'output_raw.txt'), 'r') as f:
            prompt_1 = f.read()

        prompt_2 = "Without providing any explanation, return an python integer \
                    list that has the keypoint indices that the end-effector needs to be at \
                    for each stage."
            
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": prompt_1 + ".\n" + prompt_2
                    },
                ]
            }
        ]

        stream = self.ai_client.chat.completions.create(model='gpt-4o',
                                                        messages=messages,
                                                        temperature=0.0,
                                                        max_tokens=2048,
                                                        stream=True)
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

        # save prompt
        with open(os.path.join(task_dir, 'output_raw.txt'), 'r') as f:
            prompt_1 = f.read()

        prompt_2 = "Without providing any explanation, return an python integer \
                    list that has the all the keypoints indices that this task to use "
            
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": prompt_1 + ".\n" + prompt_2
                    },
                ]
            }
        ]

        stream = self.ai_client.chat.completions.create(model='gpt-4o',
                                                        messages=messages,
                                                        temperature=0.0,
                                                        max_tokens=2048,
                                                        stream=True)
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

    def _record_video(self):

        # Functions to add point/line to image
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
                    # draw a box
                    box_width = 30 + 10 * (text_length - 1)
                    box_height = 30
                    cv2.rectangle(projected, (pixel[1] - box_width // 2, pixel[0] - box_height // 2), (pixel[1] + box_width // 2, pixel[0] + box_height // 2), (255, 255, 255), -1)
                    cv2.rectangle(projected, (pixel[1] - box_width // 2, pixel[0] - box_height // 2), (pixel[1] + box_width // 2, pixel[0] + box_height // 2), (0, 0, 0), 2)
                    # draw text
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

        # Wait until subgoal idxs is filled
        while len(self.subgoal_idxs) == 0:
            continue
        
        # Record video
        rgb = self.camera.capture_image("rgb")
        height, width, _ = rgb.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can use 'XVID' for AVI
        out = cv2.VideoWriter("saved_video.mp4", fourcc, 20, (width, height))

        while not self.terminate:
            rgb = self.camera.capture_image("rgb")

            #  ：change to xarm
            _, ee_point = self.endeffector.return_estimated_ee(self.camera, self.env.get_ee_pos())

            # Add line between point stages
            subgoal_point = self.env._keypoint_registry[self.subgoal_idxs[self.stage - 1]]["img_coord"]
            rgb = add_line_to_rgb(rgb, ee_point, subgoal_point, color=(0, 255, 0))

            # Add end effector point
            rgb = add_point_to_rgb(rgb, ee_point, color=(255, 255, 0))
            
            # Add relevant keypoints
            rgb = _project_keypoints_to_img(rgb)

            out.write(rgb)

        # Save video
        out.release()
        print(f"Video saved at saved_video.mp4!")


    def _execute(self, rekep_program_dir):
        # load metadata
        with open(os.path.join(rekep_program_dir, 'metadata.json'), 'r') as f:
            self.program_info = json.load(f)
        self.applied_disturbance = {stage: False for stage in range(1, self.program_info['num_stages'] + 1)}
        # register keypoints
        self.env.register_keypoints(self.program_info['init_keypoint_positions'], self.camera, rekep_program_dir)
        # load constraints
        self.constraint_fns = dict()
        for stage in range(1, self.program_info['num_stages'] + 1):  # stage starts with 1
            stage_dict = dict()
            for constraint_type in ['subgoal', 'path']:
                load_path = os.path.join(rekep_program_dir, f'stage{stage}_{constraint_type}_constraints.txt')
                get_grasping_cost_fn = get_callable_grasping_cost_fn(self.env)  # special grasping function for VLM to call
                stage_dict[constraint_type] = load_functions_from_txt(load_path, get_grasping_cost_fn) if os.path.exists(load_path) else []
            self.constraint_fns[stage] = stage_dict
        
        # bookkeeping of which keypoints can be moved in the optimization
        self.keypoint_movable_mask = np.zeros(self.program_info['num_keypoints'] + 1, dtype=bool)
        self.keypoint_movable_mask[0] = True  # first keypoint is always the ee, so it's movable

        # if not self.subgoal_opt:
        self.subgoal_idxs = self._get_all_subgoals(rekep_program_dir)

        # main loop
        self._update_stage(1)
        while True:
            scene_keypoints = self.env.get_keypoint_positions()
            self.keypoints = np.concatenate([[self.env.get_ee_pos()], scene_keypoints], axis=0)  # first keypoint is always the ee
            self.curr_ee_pose = self.env.get_ee_pose()
            print("Current ee pose:", self.curr_ee_pose)
            self.curr_joint_pos = self.env.get_arm_joint_positions()
            print("Current joint pos:", self.curr_joint_pos)
            self.sdf_voxels = self.env.get_sdf_voxels(self.config['sdf_voxel_size'])
            self.collision_points = self.env.get_collision_points()

            # ====================================
            # = get optimized plan
            # ====================================
            print("Stage:", self.stage)
            if self.subgoal_opt and self.is_grasp_stage== False:
                next_subgoal = self._get_next_subgoal(from_scratch=self.first_iter)
                print("Next subgoal1:", next_subgoal)
            else:  
                xyz = self.keypoints[self.subgoal_idxs[self.stage - 1]]   
                if self.is_grasp_stage:    
                    target_point = Point()
                    target_point.x = xyz[0]/1000.0
                    target_point.y = xyz[1]/1000.0
                    target_point.z = xyz[2]/1000.0       
                    # 打印发送的坐标点
                    self.get_logger().info(f"Sending target point: ({target_point.x}, {target_point.y}, {target_point.z})")
                    # 发布坐标点消息
                    self.grasp_pub.publish(target_point)

                    # 接收grasp消息
                    grasp_msg = self.wait_for_grasp_message(timeout=10.0)

                    # 当收到消息时，打印或处理数据
                    if grasp_msg:
                        self.get_logger().info(f"Received grasp pose: {grasp_msg.data}")
                    else:
                        self.get_logger().warn("Timeout occurred while waiting for grasp pose.")
                    grasp_position = np.array(grasp_msg.data[:3]) * 1000.0 # 假设位置在列表的前3个元素

                    # 旋转矩阵数据：接下来的9个值（因为旋转矩阵是3x3的矩阵，总共9个元素）
                    grasp_orientation = np.array(grasp_msg.data[3:]).reshape(3, 3)  # 将剩余的值重塑为3x3矩阵
                    # 转换为四元数
                    grasp_orientation = T.mat2quat(grasp_orientation)
                    # 打印数据
                    self.get_logger().info(f"Received grasp position: {grasp_position}")
                    self.get_logger().info(f"Received grasp orientation:\n{grasp_orientation}")
                    next_subgoal = np.concatenate([grasp_position,grasp_orientation])
                    print("Next subgoal from anygrasp:", next_subgoal)
                else:
                    next_subgoal = np.concatenate([xyz,self.curr_ee_pose[3:]])
                    print("self.subgoal_idxs[self.stage - 1]", self.subgoal_idxs[self.stage - 1])
                    print("self.keypoints", self.keypoints)
                    print("Next subgoal from keypoint:", next_subgoal)    
                # OFFSET CODE

                grasp_offset = np.array([0, 0, -10])
                subgoal_pose_homo = T.convert_pose_quat2mat(next_subgoal)
                next_subgoal[:3] += subgoal_pose_homo[:3, :3] @ grasp_offset

            print("Next subgoal:", next_subgoal)
            # input("waiting...")

            # Optimize path, otherwise do direct interpolation
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
            # self.env.sleep(15)
            if self.is_grasp_stage:
                self._execute_grasp_action()
            elif self.is_release_stage:
                self._execute_release_action()
        
            # End condition
            if self.stage == self.program_info['num_stages']: 
                self.env.sleep(2.0)
                self.terminate = True
                print("Finished!")
                return

            # progress to next stage
            self._update_stage(self.stage + 1)


    def _get_next_subgoal(self, from_scratch):
        print("getting next subgoal...")
        subgoal_constraints = self.constraint_fns[self.stage]['subgoal']
        path_constraints = self.constraint_fns[self.stage]['path']
        subgoal_pose, debug_dict = self.subgoal_solver.solve(self.curr_ee_pose,
                                                            self.keypoints,
                                                            self.keypoint_movable_mask,
                                                            subgoal_constraints,
                                                            path_constraints,
                                                            self.sdf_voxels,
                                                            self.collision_points,
                                                            self.is_grasp_stage,
                                                            self.curr_joint_pos,
                                                            from_scratch=from_scratch)
        subgoal_pose_homo = T.convert_pose_quat2mat(subgoal_pose)
        print("subgoal_pose_______",subgoal_pose)
        # if grasp stage, back up a bit to leave room for grasping
        if self.is_grasp_stage:
            subgoal_pose[:3] += subgoal_pose_homo[:3, :3] @ np.array([-self.config['grasp_depth'] / 2.0, 0, 0])
        debug_dict['stage'] = self.stage
        print_opt_debug_dict(debug_dict)
        if self.visualize:
            self.visualizer.visualize_subgoal(subgoal_pose)
        return subgoal_pose

    def _get_next_path(self, next_subgoal, from_scratch):
        print("getting next path...")
        path_constraints = self.constraint_fns[self.stage]['path']
        path, debug_dict = self.path_solver.solve(self.curr_ee_pose,
                                                    next_subgoal,
                                                    self.keypoints,
                                                    self.keypoint_movable_mask,
                                                    path_constraints,
                                                    self.sdf_voxels,
                                                    self.collision_points,
                                                    self.curr_joint_pos,
                                                    from_scratch=from_scratch)
        print_opt_debug_dict(debug_dict)
        processed_path = self._process_path(path)
        if self.visualize:
            self.visualizer.visualize_path(processed_path)
        return processed_path

    def _process_path(self, path):
        # spline interpolate the path from the current ee pose
        full_control_points = np.concatenate([
            self.curr_ee_pose.reshape(1, -1),
            path,
        ], axis=0)
        num_steps = get_linear_interpolation_steps(full_control_points[0], full_control_points[-1],
                                                    self.config['interpolate_pos_step_size'],
                                                    self.config['interpolate_rot_step_size'])
        dense_path = spline_interpolate_poses(full_control_points, num_steps)
        # add gripper action
        ee_action_seq = np.zeros((dense_path.shape[0], 8))
        ee_action_seq[:, :7] = dense_path
        ee_action_seq[:, 7] = self.env.get_gripper_null_action()
        return ee_action_seq

    def _update_stage(self, stage):
        # update stage
        self.stage = stage
        self.is_grasp_stage = self.program_info['grasp_keypoints'][self.stage - 1] != -1
        self.is_release_stage = self.program_info['release_keypoints'][self.stage - 1] != -1
        # can only be grasp stage or release stage or none
        assert self.is_grasp_stage + self.is_release_stage <= 1, "Cannot be both grasp and release stage"
        if self.is_grasp_stage:  # ensure gripper is open for grasping stage
            self.env.open_gripper()
        # clear action queue
        self.action_queue = []
        # update keypoint movable mask
        self._update_keypoint_movable_mask()
        if stage == 1:
            self.first_iter = True

    def _update_keypoint_movable_mask(self):
        for i in range(1, len(self.keypoint_movable_mask)):  # first keypoint is ee so always movable
            keypoint_object = self.env.get_object_by_keypoint(i - 1)
            # self.keypoint_movable_mask[i] = self.env.is_grasping(keypoint_object)
            if self.is_grasp_stage:  
                self.keypoint_movable_mask[self.subgoal_idxs[self.stage - 1]+1]= True
            print(f"Keypoint {i} movable mask: {self.keypoint_movable_mask[i]}")


    def _execute_grasp_action(self):
        pregrasp_pose = self.env.get_ee_pose()
        # grasp_pose = pregrasp_pose.copy()
        print("pregrasp_pose", pregrasp_pose)
        # print("pose in grasp", pose)
        pregrasp_pose[:3] += T.quat2mat(pregrasp_pose[3:]) @ np.array([0, 0, self.config['grasp_depth']])
        grasp_action = np.concatenate([pregrasp_pose, [self.env.get_gripper_close_action()]])
        grasp_action = grasp_action.reshape(1, -1)
        self.env.execute_action(grasp_action, precise=True)
    
    def _execute_release_action(self):
        self.env.open_gripper()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='pen', help='task to perform')
    parser.add_argument('--use_cached_query', action='store_true', help='instead of querying the VLM, use the cached query')
    parser.add_argument('--visualize', action='store_true', help='visualize each solution before executing (NOTE: this is blocking and needs to press "ESC" to continue)')
    args = parser.parse_args()

    task_list = {

        'block':{
            'scene_file': './configs/og_scene_file_red_pen.json',
            # 'instruction': 'pick up the red block and drop it into the box',
            'instruction': 'pick up the red block and drop it into the box',
            'rekep_program_dir': './rekep/vlm_query/2025-03-16_19-54-00_pick_up_red_block_and_drop_it_into_the_box'
            },

        'pen': {
            'scene_file': './configs/og_scene_file_red_pen.json',
            'instruction': 'pick up the black pen and drop it upright into the holder',
            'rekep_program_dir': './rekep/vlm_query/2025-03-16_19-54-00_pick_up_eraser_and_drop_it_into_the_masking_tape'
            },
        'eraser': {
            'scene_file': './configs/og_scene_file_red_pen.json',
            'instruction': 'pick up eraser and drop it into the masking tape',
            'rekep_program_dir': './rekep/vlm_query/2025-03-16_19-54-00_pick_up_eraser_and_drop_it_into_the_masking_tape'
            },
        'chess': {
            'scene_file': './configs/og_scene_file_red_pen.json',
            'instruction': 'place the white king and black king into the box. choose one keypoint for the box',
            'rekep_program_dir': './rekep/vlm_query/2025-03-16_17-16-45_place_the_white_king_and_black_king_into_the_box._choose_one_keypoint_for_the_box'
            },
        'stack': {
            'scene_file': './configs/og_scene_file_red_pen.json',
            'instruction': 'place the three small cubes onto the respective large cubes with similar color',
            'rekep_program_dir': './rekep/vlm_query/2025-03-15_17-03-59_place_the_three_small_cubes_onto_the_respective_large_cubes_with_similar_color'
        },
    }
    
    # Initialize ROS2
    rclpy.init(args=None)
    
    try:
        task = task_list['block']
        scene_file = task['scene_file']
        instruction = task['instruction']
        main = MainRekepNode(scene_file, visualize=args.visualize)
        main.perform_task(instruction,
                        rekep_program_dir=task['rekep_program_dir'] if args.use_cached_query else None)
    finally:
        if 'main' in locals():
            main.destroy_node()
        rclpy.shutdown()