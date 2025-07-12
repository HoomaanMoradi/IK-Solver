import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List
import os

class Visualizer:
    def __init__(self, robot_urdf_path: str, linear_axis_urdf_path: str, use_gui: bool = True):
        self.use_gui = use_gui
        self.robot_urdf_path = robot_urdf_path
        self.linear_axis_urdf_path = linear_axis_urdf_path
        self.robot_id = None
        self.linear_axis_id = None
        self.joint_indices = []
        self.linear_joint_idx = -1
        self.end_effector_link_index = -1
        self.end_effector_history = []
        self.p = p
        if self.use_gui:
            self._init_pybullet()

    def _init_pybullet(self):
        """Initialize PyBullet simulation and load robots from URDF."""
        try:
            if self.use_gui:
                self.p.connect(p.GUI)
                self.p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
                self.p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
                self.p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
                
                
            else:
                self.p.connect(p.DIRECT)
            
            self.p.setGravity(0, 0, -9.81)
            self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.p.loadURDF("plane.urdf")
            
            # Load the linear axis with a fixed base
            self.linear_axis_id = self.p.loadURDF(self.linear_axis_urdf_path, useFixedBase=True)
            
            # Find the end link of the linear axis to attach the robot to
            num_joints_linear = self.p.getNumJoints(self.linear_axis_id)
            linear_axis_end_link_index = -1
            for i in range(num_joints_linear):
                info = self.p.getJointInfo(self.linear_axis_id, i)
                # We assume the last link is the one to attach to
                linear_axis_end_link_index = i

            robot_start_pos = [0, 0, 0] # Default fallback
            if linear_axis_end_link_index != -1:
                linear_axis_end_link_state = self.p.getLinkState(self.linear_axis_id, linear_axis_end_link_index)
                linear_axis_end_link_pos = linear_axis_end_link_state[0]
                linear_axis_end_link_orn = linear_axis_end_link_state[1]

                # Calculate the desired robot base position relative to the linear axis end link
                offset_in_linear_axis_frame = [0.1, 0, 0] # The desired offset
                robot_start_pos, _ = self.p.multiplyTransforms(
                    linear_axis_end_link_pos,
                    linear_axis_end_link_orn,
                    offset_in_linear_axis_frame,
                    [0, 0, 0, 1] # Identity quaternion for orientation offset
                )

            # Load the robot with a non-fixed base so it can be attached to the linear axis
            self.robot_id = self.p.loadURDF(self.robot_urdf_path, basePosition=robot_start_pos, useFixedBase=False)

            # Change color of the robot and linear axis
            for i in range(p.getNumJoints(self.robot_id)):
                p.changeVisualShape(self.robot_id, i, rgbaColor=[0.8, 0.8, 0.8, 1])
            p.changeVisualShape(self.robot_id, -1, rgbaColor=[0.8, 0.8, 0.8, 1]) # Base link

            for i in range(p.getNumJoints(self.linear_axis_id)):
                p.changeVisualShape(self.linear_axis_id, i, rgbaColor=[0.6, 0.6, 0.6, 1])
            p.changeVisualShape(self.linear_axis_id, -1, rgbaColor=[0.6, 0.6, 0.6, 1]) # Base link

            if linear_axis_end_link_index != -1:
                # Attach the robot to the moving part of the linear axis
                self.p.createConstraint(
                    parentBodyUniqueId=self.linear_axis_id,
                    parentLinkIndex=linear_axis_end_link_index, # The moving link of the linear axis
                    childBodyUniqueId=self.robot_id,
                    childLinkIndex=-1,  # Base of the robot
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0], # Now 0,0,0 as robot is loaded at correct offset
                    childFramePosition=[0, 0, 0]
                )

            # Get joint indices for the robot (revolute joints)
            self.joint_indices = []
            for i in range(self.p.getNumJoints(self.robot_id)):
                info = self.p.getJointInfo(self.robot_id, i)
                if info[2] == p.JOINT_REVOLUTE:
                    self.joint_indices.append(i)
                    self.p.resetJointState(self.robot_id, i, 0) # Reset robot joint to 0

            # Get joint index for the linear axis (prismatic joint)
            self.linear_joint_idx = -1
            for i in range(self.p.getNumJoints(self.linear_axis_id)):
                 info = self.p.getJointInfo(self.linear_axis_id, i)
                 if info[2] == p.JOINT_PRISMATIC:
                     self.linear_joint_idx = i
                     self.p.resetJointState(self.linear_axis_id, i, 0) # Reset linear axis joint to 0
                     break
            
            # Get end-effector link index
            self.end_effector_link_index = self.joint_indices[-1]

            if self.use_gui:
                self.p.resetDebugVisualizerCamera(
                    cameraDistance=5,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=[2, 0, 1]
                )
            
            # Perform a few simulation steps to allow the system to settle
            for _ in range(100):
                self.p.stepSimulation()
                
            return True
            
        except Exception as e:
            print(f"Error initializing PyBullet: {e}")
            return False

    def visualize_pybullet(self, joint_angles: List[float], linear_pos: float):
        """Visualize the robot in PyBullet by setting joint positions."""
        if not self.use_gui or self.robot_id is None or self.linear_axis_id is None:
            return

        # Set the position of the linear axis
        if self.linear_joint_idx != -1:
            self.p.setJointMotorControl2(
                bodyUniqueId=self.linear_axis_id,
                jointIndex=self.linear_joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=linear_pos,
                force=10000, # Increased force for stability
                maxVelocity=5 # Set max velocity for linear axis
            )

        # Set the positions of the robot's joints
        for i, joint_index in enumerate(self.joint_indices):
            if i < len(joint_angles):
                self.p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_angles[i],
                    force=10000, # Increased force for faster movement, matching linear axis
                    maxVelocity=1 # Increased speed
                )
        
        self.p.stepSimulation()
        if self.end_effector_link_index != -1:
            link_state = self.p.getLinkState(self.robot_id, self.end_effector_link_index)
            self.end_effector_history.append(link_state[0])

    def is_connected(self):
        return self.p.isConnected()

    def close_pybullet(self):
        """Close the PyBullet simulation."""
        if self.is_connected():
            self.p.disconnect()

    def plot_end_effector_trajectory(self, start_pos=None, target_pos=None):
        """
        Plot the end-effector trajectory in a 3D graph with optional start and target points.
        
        Args:
            start_pos: Optional [x, y, z] coordinates of the start position
            target_pos: Optional [x, y, z] coordinates of the target position
        """
        if not self.end_effector_history:
            print("No end-effector history to plot.")
            return

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        x = [pos[0] for pos in self.end_effector_history]
        y = [pos[1] for pos in self.end_effector_history]
        z = [pos[2] for pos in self.end_effector_history]
        
        ax.plot(x, y, z, 'b-', linewidth=2, label='End-Effector Trajectory')
        
        # Plot start and end points of the trajectory
        if len(self.end_effector_history) > 0:
            start = self.end_effector_history[0]
            end = self.end_effector_history[-1]
            ax.scatter(start[0], start[1], start[2], c='g', s=100, marker='o', label='Trajectory Start')
            ax.scatter(end[0], end[1], end[2], c='r', s=100, marker='x', label='Trajectory End')
        
        # Plot provided start and target positions if available
        if start_pos is not None:
            ax.scatter(start_pos[0], start_pos[1], start_pos[2], 
                      c='lime', s=150, marker='*', edgecolors='black', 
                      linewidth=1, label='Initial Position')
        
        if target_pos is not None:
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], 
                      c='red', s=150, marker='*', edgecolors='black', 
                      linewidth=1, label='Target Position')
        
        # Set labels and title
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_zlabel('Z (m)', fontsize=12)
        ax.set_title('End-Effector Trajectory with Start and Target Points', fontsize=14)
        
        # Add grid and legend
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend(loc='upper right', fontsize=10)
        
        # Set equal aspect ratio
        max_range = max(max(x)-min(x), max(y)-min(y), max(z)-min(z)) * 0.6
        mid_x = (max(x) + min(x)) * 0.5
        mid_y = (max(y) + min(y)) * 0.5
        mid_z = (max(z) + min(z)) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Save and close
        plt.tight_layout()
        plt.savefig('end_effector_trajectory.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
