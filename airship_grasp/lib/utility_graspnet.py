import logging
import os
import yaml

from graspnetAPI.grasp import GraspGroup
import numpy as np
import open3d as o3d
import torch

from lib.camera_intrinsic import CameraIntrinsic
from lib.Scale_Balanced_Grasp.models.graspnet import GraspNet_MSCQ, pred_decode
from lib.Scale_Balanced_Grasp.utils.collision_detector import ModelFreeCollisionDetector
from lib.Scale_Balanced_Grasp.utils.data_utils import create_point_cloud_from_depth_image

class UtilityGraspNet:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        # Depth camera intrinsic
        self.model_checkpoint_path = self.config.get('model_checkpoint_path')
        self.expand_mask_width = self.config.get('expand_mask_width')
        self.expand_mask_height = self.config.get('expand_mask_height')
        self.num_points = self.config.get('num_points')
        self.num_grasp_pose = self.config.get('num_grasp_pose')
        self.debug_mode = self.config.get('debug_mode')
        # Load camera intrinsic parameters
        self.camera_info = CameraIntrinsic(config_path)
        # Set the device
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # Initialize the GraspNet model
        self.net = self.initialize_graspnet()
        
    def initialize_graspnet(self):
        """Initialize the GraspNet model and load the checkpoint."""
        # Initialize the network
        net = GraspNet_MSCQ(
            input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False
        )
        net.to(self.device)
        # Load the checkpoint
        checkpoint_path = self.model_checkpoint_path
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(f"Checkpoint file not found at {checkpoint_path}")
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        logging.info(f'-> loaded checkpoint {checkpoint_path} (epoch: {start_epoch})')
        # Set the model to evaluation mode
        net.eval()
        return net
        
    def create_centered_box_mask(self, original_mask, width, height):
        """Create a centered rectangular mask based on the original mask."""
        mask_bool = original_mask > 0
        rows, cols = np.where(mask_bool)
        if not rows.size or not cols.size:
            raise ValueError("No match mask")
        center_row = int(np.mean(rows))
        center_col = int(np.mean(cols))
        new_mask = np.zeros_like(mask_bool, dtype=np.uint8)
        row_start = max(center_row - height // 2, 0)
        row_end = min(center_row + height // 2 + 1, mask_bool.shape[0])
        col_start = max(center_col - width // 2, 0)
        col_end = min(center_col + width // 2 + 1, mask_bool.shape[1])
        new_mask[row_start:row_end, col_start:col_end] = 255
        return new_mask
    
    def get_3d_center_mask(self, original_mask, depth, use_isaac_sim):
        """Compute the 3D center point of the mask."""
        mask_bool = original_mask > 0
        rows, cols = np.where(mask_bool)
        center_row = int(np.mean(rows))
        center_col = int(np.mean(cols))
        center_depth = depth[center_row, center_col]
        if use_isaac_sim:
            points_z = center_depth
        else:
            points_z = center_depth / self.camera_info.scale
        points_x = (center_row - self.camera_info.cx) * points_z / self.camera_info.fx
        points_y = (center_col - self.camera_info.cy) * points_z / self.camera_info.fy
        return np.stack([points_x, points_y, points_z], axis=-1)

    def get_and_process_data(self, rgb, depth, mask, use_isaac_sim):
        """Process RGB, depth, and mask data to prepare for grasp prediction."""
        color = rgb
        workspace_mask = mask
        if use_isaac_sim:
            depth = depth * self.camera_info.scale
        cloud = create_point_cloud_from_depth_image(depth, self.camera_info, organized=True)
        new_mask = self.create_centered_box_mask(workspace_mask, self.expand_mask_width, self.expand_mask_height)
        mask = (new_mask > 0) & (depth > 0)
        cloud_masked = cloud[mask]
        color_masked = color[mask]
        # Sample points if there are more than 20000, else sample with replacement
        if len(cloud_masked) >= self.num_points:
            idxs = np.random.choice(len(cloud_masked), self.num_points, replace=False)
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(len(cloud_masked), self.num_points - len(cloud_masked), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]
        # Convert data to PyTorch tensors
        cloud_pcd = o3d.geometry.PointCloud()
        cloud_pcd.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud_pcd.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
        cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
        cloud_sampled = cloud_sampled.to(self.device)
        # Generate end points
        end_points = {
            'point_clouds': cloud_sampled,
            'cloud_colors': color_sampled
        }
        return end_points, cloud_pcd

    def get_grasps(self, end_points):
        """Generate grasps from the network output."""
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        grasps_array = grasp_preds[0].detach().cpu().numpy()
        grasps = GraspGroup(grasps_array)
        return grasps

    def collision_detection(self, grasp_pose, cloud):
        """Filter out grasps that result in collisions."""
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=0.01)
        collision_mask = mfcdetector.detect(grasp_pose, approach_dist=0.05, collision_thresh=0.01)
        grasp_pose = grasp_pose[~collision_mask]
        return grasp_pose

    def grasp_pose_estimation(self, rgb, depth, mask):
        """Estimate grasp poses from RGB, depth, and mask images."""
        end_points, cloud = self.get_and_process_data(rgb, depth, mask)
        grasp_pose = self.get_grasps(end_points)
        grasp_pose = self.collision_detection(grasp_pose, np.array(cloud.points))
        grasp_pose.nms()
        grasp_pose.sort_by_score()
        grasp_pose = grasp_pose[:self.num_grasp_pose]
        # visualize results
        if self.debug_mode:
            grippers = grasp_pose.to_open3d_geometry_list()
            o3d.visualization.draw_geometries([cloud, *grippers]) 
        return grasp_pose
