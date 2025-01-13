import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

class UtilityIk():
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        self.min_angle_selcet = self.config.get('min_angle_select')
        self.max_angle_select = self.config.get('max_angle_select')
        # Load T_cam2real_gripper, T_depth2rgb, T_est2virtual_gripper
        self.cam2real_gripper_xyzrpy = self.config.get('cam2real_gripper_xyzrpy')
        self.est2virtual_gripper_xyzrpy = self.config.get('est2virtual_gripper_xyzrpy')

    def is_out_of_reach(self, target_x, target_y, target_z):
        """
        Update the target joint based on the given position.
        :param target_x: Target X coordinate
        :param target_y: Target Y coordinate
        :param target_z: Target Z coordinate
        :return: True if the target is within range and updated, False otherwise
        """
        if not (0.24 <= target_x <= 0.6 and -0.4 <= target_y <= 0.4 and -0.55 <= target_z <= 0.3):
            logging.info("Target position out of range")
            return True
        else:
            logging.info("Target position inside range")
            return False

    def inverse(self, x, y, z):
        rx = np.sqrt(x**2 + y**2) - 0.095
        rz = z + 0.1693 - 0.22934
        l = np.sqrt(rx**2 + rz**2)
        if l > 0.27 + 0.267:
            logging.info('Out of reach')
            return None
        dz = 0.0725
        theta1 = np.arctan2(y, x) - np.arcsin(dz/l)
        alpha = np.arctan2(rz, rx)
        beta = np.arccos((0.27**2 + l**2 - 0.267**2) / (2*0.27*l))
        theta2 = -alpha - beta
        theta3 = np.pi - np.arccos((0.27**2 + 0.267**2 - l**2) / (2*0.27*0.267))
        theta4 = - theta2 - theta3 - np.pi/2.0
        theta5 = - np.pi/2
        theta6 =  theta1 + np.pi - (67.5/180.0)*np.pi
        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def excute(self, target_x, target_y, target_z):
        target_joint_states = {}
        joint_names = [
            'joint_link1_to_base',
            'joint_link2_to_link1',
            'joint_link3_to_link2',
            'joint_link4_to_link3',
            'joint_link5_to_link4',
            'joint_link6_to_link5'
        ]
        logging.info(f"Calling inverse with x: {target_x}, y: {target_y}, z: {target_z}")
        joint_angles = self.inverse(target_x, target_y, target_z)
        if(joint_angles is None):
            logging.info('Out of reach')
            return target_joint_states
        else:
            for i, joint_name in enumerate(joint_names):
                target_joint_states[joint_name] = joint_angles[i]

        logging.info(f"Target joint states: {target_joint_states}")
        return target_joint_states

    def pose_to_transformation_matrix(self, arm_pose):
        """Convert pose [x, y, z, rx, ry, rz] to a transformation matrix."""
        x, y, z, rx, ry, rz = arm_pose
        rotation = R.from_euler('ZYX', [rz, ry, rx], degrees=True)
        r_matrix = rotation.as_matrix()
        T = np.eye(4)
        T[:3, :3] = r_matrix
        T[:3, 3] = [x, y, z]
        return T

    def tf_to_transformation_matrix(self, arm_pose):
        """
        Convert pose (position and orientation) into a 4x4 homogeneous transformation matrix
        """
        # Position
        position = arm_pose['position']

        # Rotation
        rotation_matrix = arm_pose['orientation'].as_matrix()  # Convert the rotation object to a rotation matrix

        # Construct the homogeneous transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = position

        return transformation_matrix

    def matrix_to_euler_and_translation(self, T):
        """Convert transformation matrix to pose [x, y, z, rx, ry, rz]."""
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        r = R.from_matrix(rotation_matrix)
        euler_angles = r.as_euler('ZYX', degrees=True)
        x, y, z = translation
        rz, ry, rx = euler_angles
        return [x, y, z, rx, ry, rz]

    def select_best_grasp(self, grasp_pose, center_mask_point, trans_real2base):
        """Select the best grasp based on distance to the mask center and valid angles."""
        # Transformations
        trans_cam2real_gripper = self.pose_to_transformation_matrix(self.cam2real_gripper_xyzrpy)
        trans_est2virtual_gripper = self.pose_to_transformation_matrix(self.est2virtual_gripper_xyzrpy)
        distances = []
        # Compute distance from the mask center for each candidate grasp pose
        for i in range(len(grasp_pose.rotation_matrices)):
            trans_virtual2cam = np.eye(4)
            trans_virtual2cam[:3, :3] = grasp_pose.rotation_matrices[i]
            trans_virtual2cam[:3, 3] = grasp_pose.translations[i]
            grasp_coords = self.matrix_to_euler_and_translation(trans_virtual2cam)
            distance = np.sqrt(np.sum((grasp_coords[:3] - center_mask_point) ** 2))
            distances.append(distance)
        # Initialize variables
        min_distance_index = np.argmin(np.array(distances))
        attempt = 0
        found_valid_grasp = False
        # Try to find a valid grasp within the angular constraints
        while not found_valid_grasp and attempt < len(distances):
            center_mask_grasp = grasp_pose[min_distance_index:min_distance_index + 1]
            # Compute the transformation matrix from the virtual frame to the object grasp pose
            trans_virtual2cam = np.eye(4)
            trans_virtual2cam[:3, :3] = center_mask_grasp.rotation_matrices[0]
            trans_virtual2cam[:3, 3] = center_mask_grasp.translations[0]
            trans_est2base = trans_real2base @ trans_cam2real_gripper @ trans_virtual2cam @ trans_est2virtual_gripper
            # Get grasp coordinates and check if the angles are within the desired range
            grasp_coords = self.matrix_to_euler_and_translation(trans_est2base)
            center_mask_euler_angles = grasp_coords[3:]
            if self.min_angle_selcet <= center_mask_euler_angles[2] <= self.max_angle_select:
                found_valid_grasp = True
            else:
                # Mark this grasp as invalid and try the next best candidate
                distances[min_distance_index] = np.inf
                min_distance_index = np.argmin(np.array(distances))
                attempt += 1
        # Convert coordinates from meters to millimeters
        grasp_coords[:3] = [x * 1000.0 for x in grasp_coords[:3]]
        if not found_valid_grasp:
            logging.info('Transform extrapolation error: No valid grasp found within constraints.')
        return grasp_coords

    def get_init_pose(self):
        """
        Get the initial joint pose
        :return: Initial joint position dictionary, format similar to excute function
        """
        target_joint_states = {}
        joint_names = [
            'joint_link1_to_base',
            'joint_link2_to_link1',
            'joint_link3_to_link2',
            'joint_link4_to_link3',
            'joint_link5_to_link4',
            'joint_link6_to_link5'
        ]

        # Set the initial joint positions
        joint_state_positions = [
            0.0,    # joint_link1_to_base
            -0.5,   # joint_link2_to_link1
            0.7,    # joint_link3_to_link2
            -1.7,   # joint_link4_to_link3
            -1.57,  # joint_link5_to_link4
            1.9     # joint_link6_to_link5
        ]

        # Combine joint names and positions into target joint state dictionary
        for i, joint_name in enumerate(joint_names):
            target_joint_states[joint_name] = joint_state_positions[i]

        # Return target joint states
        return target_joint_states

    def get_raise_pose(self):
        """
        Get the raised joint pose
        :return: Raised joint position dictionary, format similar to get_init_pose
        """
        target_joint_states = {}
        joint_names = [
            'joint_link1_to_base',
            'joint_link2_to_link1',
            'joint_link3_to_link2',
            'joint_link4_to_link3',
            'joint_link5_to_link4',
            'joint_link6_to_link5'
        ]

        # Set the raised joint positions
        joint_state_positions = [
            0.0,    # joint_link1_to_base
            -1.57,  # joint_link2_to_link1
            0.35,   # joint_link3_to_link2
            -1.04,  # joint_link4_to_link3
            -1.57,  # joint_link5_to_link4
            2.18    # joint_link6_to_link5
        ]

        # Combine joint names and positions into target joint state dictionary
        for i, joint_name in enumerate(joint_names):
            target_joint_states[joint_name] = joint_state_positions[i]

        # Return target joint states
        return target_joint_states
