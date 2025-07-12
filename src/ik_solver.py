import numpy as np
import pybullet as p
from typing import List, Tuple, Optional
from src.urdf_parser import URDFParser
from src.visualizer import Visualizer

class IKSolver7DOF:
    """Inverse Kinematics solver for 6-DOF robot on linear axis."""

    def __init__(self, robot_urdf_path: str, linear_axis_urdf_path: str, visualizer: Visualizer):
        self.visualizer = visualizer
        with open(robot_urdf_path, 'r') as f:
            robot_urdf_content = f.read()
        with open(linear_axis_urdf_path, 'r') as f:
            linear_axis_urdf_content = f.read()

        self.robot_parser = URDFParser(robot_urdf_content)
        self.linear_parser = URDFParser(linear_axis_urdf_content)
        self._extract_robot_parameters()

    def _extract_robot_parameters(self):
        """Extract robot parameters from URDF data."""
        self.joint_limits = self.robot_parser.get_joint_limits()
        linear_joints = [j for j in self.linear_parser.joints.values() if j['type'] == 'prismatic']
        if linear_joints:
            linear_joint = linear_joints[0]
            self.linear_range = (linear_joint['limits'][0], linear_joint['limits'][1])
            self.linear_offset = np.array(linear_joint['origin_xyz'])
        else:
            self.linear_range = (-0.41, 7.3)
            self.linear_offset = np.array([0.6, 0.43, 0.55])

    def inverse_kinematics(self, target_pos: np.ndarray, target_orn_rpy: np.ndarray) -> List[Tuple[List[float], float]]:
        """
        Solve inverse kinematics for the 7-DOF system by iterating over the linear axis.

        Args:
            target_pos: Target position [x, y, z] in world coordinates.
            target_orn_rpy: Target orientation [roll, pitch, yaw] in radians.

        Returns:
            A list of (joint_angles, linear_pos) solutions.
        """
        solutions = []
        target_orn_quat = self.visualizer.p.getQuaternionFromEuler(target_orn_rpy)

        # Iterate over a few discrete positions of the linear axis for a basic search
        for linear_pos in np.linspace(self.linear_range[0], self.linear_range[1], 20):
            # Calculate the robot base position for this linear axis position
            robot_base_pos = np.array([linear_pos, 0, 0]) + self.linear_offset
            
            # Transform the target pose from the world frame to the robot's base frame
            target_pos_in_robot_frame = target_pos - robot_base_pos
            
            # Solve the 6-DOF IK for the arm
            joint_angles = self.visualizer.p.calculateInverseKinematics(
                bodyUniqueId=self.visualizer.robot_id,
                endEffectorLinkIndex=self.visualizer.end_effector_link_index,
                targetPosition=target_pos_in_robot_frame,
                targetOrientation=target_orn_quat,
                solver=0,  # Use the default Damped Least Squares solver
                maxNumIterations=100,
                residualThreshold=0.01
            )
            
            # PyBullet returns a tuple, we only need the joint angles for the revolute joints
            if joint_angles and len(joint_angles) >= len(self.visualizer.joint_indices):
                # A solution was found, let's check if it's within joint limits
                valid_solution = True
                for i, angle in enumerate(joint_angles[:len(self.joint_limits)]):
                    lower, upper = self.joint_limits[i]
                    if not (lower <= angle <= upper):
                        valid_solution = False
                        break
                
                if valid_solution:
                    solutions.append((list(joint_angles[:len(self.visualizer.joint_indices)]), linear_pos))

        return solutions

    def find_best_solution(self, solutions: list,
                           current_joints: Optional[List[float]] = None,
                           current_linear: Optional[float] = None,
                           target_pos: Optional[np.ndarray] = None,
                           target_orn_rpy: Optional[np.ndarray] = None) -> Optional[Tuple[List[float], float]]:
        """
        Find the best IK solution based on position and orientation error.
        
        Args:
            solutions: List of (joint_angles, linear_pos) solutions
            current_joints: Current joint angles (optional, used for tie-breaking)
            current_linear: Current linear position (optional, used for tie-breaking)
            target_pos: Target position [x, y, z] in world coordinates (required for error calculation)
            target_orn_rpy: Target orientation [roll, pitch, yaw] in radians (required for error calculation)
            
        Returns:
            The best solution as (joint_angles, linear_pos) or None if no solutions
        """
        if not solutions:
            return None
            
        # If no target pose is provided, fall back to the first solution
        if target_pos is None or target_orn_rpy is None:
            return solutions[0]
            
        best_solution = None
        min_error = float('inf')
        
        # Convert target orientation to quaternion for comparison
        target_quat = self.visualizer.p.getQuaternionFromEuler(target_orn_rpy)
        
        for joint_angles, linear_pos in solutions:
            # Set the robot state to this solution
            robot_base_pos = np.array([linear_pos, 0, 0]) + self.linear_offset
            
            # Calculate forward kinematics to get actual end-effector pose
            self.visualizer.set_robot_state(joint_angles, robot_base_pos)
            
            # Get current end-effector state
            ee_state = self.visualizer.p.getLinkState(
                self.visualizer.robot_id,
                self.visualizer.end_effector_link_index
            )
            ee_pos = np.array(ee_state[4])  # World position of the end-effector
            ee_quat = np.array(ee_state[5])  # World orientation of the end-effector
            
            # Calculate position error (Euclidean distance)
            pos_error = np.linalg.norm(ee_pos - target_pos)
            
            # Calculate orientation error (angle between quaternions)
            dot = np.dot(ee_quat, target_quat)
            orn_error = 1.0 - dot**2  # This gives a value between 0 and 1
            
            # Combined error with weights (adjust weights as needed)
            pos_weight = 1.0
            orn_weight = 0.5  # You can adjust this weight based on your needs
            total_error = pos_weight * pos_error + orn_weight * orn_error
            
            # If we have a tie, prefer solutions closer to the current configuration
            if abs(total_error - min_error) < 1e-6 and current_joints is not None and current_linear is not None:
                joint_dist = np.sum((np.array(joint_angles) - np.array(current_joints))**2)
                linear_dist = (linear_pos - current_linear)**2
                total_dist = joint_dist + linear_dist
                
                if total_dist < min_dist:
                    min_dist = total_dist
                    best_solution = (joint_angles, linear_pos)
            elif total_error < min_error:
                min_error = total_error
                best_solution = (joint_angles, linear_pos)
                min_dist = float('inf')  # Reset min_dist for the new best solution
                
        return best_solution
