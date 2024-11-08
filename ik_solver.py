"""
Adapted from OmniGibson and the Lula IK solver
"""
import pybullet
import pybullet_data
import numpy as np
from transform_utils import mat2quat


class IKSolver:
    """
    Class for thinly wrapping PyBullet IK solver
    """

    def __init__(
        self,
        config,
        robot_urdf_path,
        eef_name,
        reset_joint_pos,
        world2robot_homo,
    ):
        # Create robot description, kinematics, and config
        self.eef_name = eef_name
        self.reset_joint_pos = reset_joint_pos
        self.world2robot_homo = world2robot_homo
        self.config = config
        # initiate with DIRECT mode PyBullet
        pybullet.connect(pybullet.setAdditionalSearchPath)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = pybullet.loadURDF(
            robot_urdf_path, 
            basePosition=self.world2robot_homo[:3, 3],
            baseOrientation=mat2quat(world2robot_homo[:3, :3]))

    def solve(
        self,
        target_pose_homo,
        position_tolerance=0.01,
        orientation_tolerance=0.05,
        position_weight=1.0,
        orientation_weight=0.05,
        max_iterations=150,
        initial_joint_pos=None,
    ):
        """
        Backs out joint positions to achieve desired @target_pos and @target_quat

        Args:
            target_pose_homo (np.ndarray): [4, 4] homogeneous transformation matrix of the target pose in world frame
            position_tolerance (float): Maximum position error (L2-norm) for a successful IK solution
            orientation_tolerance (float): Maximum orientation error (per-axis L2-norm) for a successful IK solution
            position_weight (float): Weight for the relative importance of position error during CCD
            orientation_weight (float): Weight for the relative importance of position error during CCD
            max_iterations (int): Number of iterations used for each cyclic coordinate descent.
            initial_joint_pos (None or n-array): If specified, will set the initial cspace seed when solving for joint
                positions. Otherwise, will use self.reset_joint_pos

        Returns:
            ik_results (lazy.lula.CyclicCoordDescentIkResult): IK result object containing the joint positions and other information.
        """
        # convert target pose to robot base frame
        # target_pose_robot = np.dot(self.world2robot_homo, target_pose_homo)
        target_pose_pos = target_pose_homo[:3, 3]
        target_pose_rot = target_pose_homo[:3, :3]
        
        ik_results = pybullet.calculateInverseKinematics(
            self.robot,
            self.eef_name,
            targetPosition=target_pose_pos,
            targetOrientation=mat2quat(target_pose_rot),
            lowerLimits=self.config['joint_lower_limit'],
            upperLimits=self.config['joint_upper_limit'],
            maxNumIterations=max_iterations,
        )
        
        return ik_results