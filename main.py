import numpy as np
import os
import sys
import time
import argparse
import logging
from src.ik_solver import IKSolver7DOF
from src.visualizer import Visualizer

def get_user_pose_from_args():
    """Get the target pose from command-line arguments or use defaults."""
    parser = argparse.ArgumentParser(description='Robot IK Solver')
    parser.add_argument('--position', type=float, nargs=3, default=[5.0, -1.5, 1.0],
                        help='Target position (X Y Z)')
    parser.add_argument('--orientation', type=float, nargs=3, default=[0, 90.0, -90.0],
                        help='Target orientation in degrees (Roll Pitch Yaw)')

    # Check if arguments were passed (besides script name)
    if len(sys.argv) > 1:
        args = parser.parse_args()
    else:
        args = parser.parse_args([])  # parse empty args to trigger defaults

    orientation_rad = np.deg2rad(args.orientation)
    return np.array(args.position), orientation_rad

if __name__ == "__main__":
    # Create log directory if it doesn't exist
    log_dir = 'log'
    os.makedirs(log_dir, exist_ok=True)

    # Configure logging for file
    file_handler = logging.FileHandler(os.path.join(log_dir, 'robot_ik.log'))
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)

    # Configure main logger (for console and file)
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(logging.StreamHandler()) # For console output

    # Configure a separate logger for iteration data (only to file)
    iteration_logger = logging.getLogger('iteration_logger')
    iteration_logger.setLevel(logging.INFO)
    iteration_logger.propagate = False # Prevent messages from going to root logger
    iteration_logger.addHandler(file_handler) # Only log to file

    robot_urdf_path = os.path.join('robot-urdfs', 'abb_irb6700_150_320', 'abb_irb6700_150_320.urdf')
    linear_axis_urdf_path = os.path.join('robot-urdfs', 'linear_axis', 'linear_axis.urdf')
    
    visualizer = Visualizer(robot_urdf_path, linear_axis_urdf_path, use_gui=True)
    ik_solver = IKSolver7DOF(robot_urdf_path, linear_axis_urdf_path, visualizer)
    
    try:
        target_position, target_orientation = get_user_pose_from_args()
        
        logging.info(f"Solving IK for target position: {target_position} and orientation: {target_orientation}")
        solutions = ik_solver.inverse_kinematics(target_position, target_orientation)
        
        if solutions:
            logging.info(f"Found {len(solutions)} solutions. Visualizing the best one.")
            
            best_solution = ik_solver.find_best_solution(
                solutions,
                current_joints=[0.0] * 6,
                current_linear=0.0
            )

            if best_solution:
                joints, linear = best_solution
                logging.info(f"Best Solution: Joints: {[f'{j:.3f}' for j in joints]}, Linear: {linear:.3f}")
                logging.info(f"Visualizing solution in PyBullet...")
                # Allow robot to settle into initial pose
                for _ in range(100):
                    visualizer.visualize_pybullet(joints, linear)
                    time.sleep(1./240.)

                for i in range(int(5.0 * 240)):
                    visualizer.visualize_pybullet(joints, linear)
                    # Log joint and linear axis positions at each iteration (only to file)
                    iteration_logger.info(f"Iteration {i+1}: Joints: {[f'{j:.3f}' for j in joints]}, Linear: {linear:.3f}")
                    time.sleep(1./240.)
                
                logging.info("Robot has reached the target. Plotting end effector trajectory...")
                
                # Get the start position (initial end-effector position)
                start_pos = visualizer.end_effector_history[0] if visualizer.end_effector_history else None
                
                # Plot the trajectory with start and target positions
                visualizer.plot_end_effector_trajectory(
                    start_pos=start_pos,
                    target_pos=target_position
                )
                
                logging.info("The simulation will remain open.")
                logging.info("Close the PyBullet window to exit.")
                while visualizer.is_connected():
                    time.sleep(0.1)

            else:
                logging.warning("Could not find a best solution.")
        else:
            logging.warning("No IK solutions found.")

    except KeyboardInterrupt:
        logging.info("Simulation interrupted by user.")
    finally:
        # Get the start position (initial end-effector position) if available
        start_pos = visualizer.end_effector_history[0] if visualizer.end_effector_history else None
        
        # Plot the trajectory with start and target positions if target_position is defined
        if 'target_position' in locals():
            visualizer.plot_end_effector_trajectory(
                start_pos=start_pos,
                target_pos=target_position
            )
        else:
            visualizer.plot_end_effector_trajectory()
            
        visualizer.close_pybullet()