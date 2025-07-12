# 7-DOF Robot Inverse Kinematics Solver

This project implements an Inverse Kinematics (IK) solver for a 7-degree-of-freedom (DOF) robotic system, which consists of a 6-DOF robotic arm mounted on a linear axis. The solver finds the joint configurations required to position the robot's end-effector at a specified target position and orientation. The project includes a visualization tool using PyBullet to simulate the robot's movement in a 3D environment and plots the end-effector trajectory upon reaching the target.

## Features

-   **URDF Parsing:** The solver parses URDF files to extract robot parameters, including joint limits, link origins, and other properties.
-   **7-DOF Inverse Kinematics:** It provides a framework for solving the IK problem for a 7-DOF robot, combining the linear axis position with the 6-DOF arm's joint angles.
-   **PyBullet Visualization:** The project includes a visualization tool to simulate the robot's behavior in a realistic 3D environment. The visualization features include:
    -   Real-time simulation for smooth motion.
    -   Shadows and realistic rendering.
    -   Customizable colors for the robot and linear axis.
-   **End-Effector Trajectory Plotting:** The script now plots the end-effector's trajectory in a 3D graph using Matplotlib, providing a clear visualization of its path.
-   **Modular Design:** The code is organized into two main classes:
    -   `URDFParser`: Handles the parsing of URDF files.
    -   `IKSolver7DOF`: Implements the IK solver and PyBullet visualization.

## How It Works

The core of the project is the `IKSolver7DOF` class, which integrates the robot's kinematics with the PyBullet simulation. The solver takes a target position and orientation for the end-effector and computes the required joint angles for the 6-DOF arm and the position of the linear axis.

The `visualize_pybullet` function allows you to see the robot's movement in real-time. It sets the joint positions and linear axis position and then runs the simulation for a specified duration.


### What the script does:
1. Initializes the IK solver with the robot and linear axis URDF files
2. Solves the inverse kinematics for the specified target position and orientation
3. Visualizes the robot's motion in PyBullet
4. Plots the end-effector's trajectory in 3D when the target is reached
5. Saves the trajectory plot as `end_effector_trajectory.png`

### Arguments:
- `--position X Y Z`: Target position in meters (X, Y, Z coordinates)
- `--orientation ROLL PITCH YAW`: Target orientation in degrees (Euler angles)

## Installation

1. **Set up a virtual environment (recommended):**
   ```bash
   # Create a new virtual environment
   python -m venv .venv
   
   # Activate the virtual environment
   # On Windows:
   .\venv\Scripts\activate
   # On Unix or MacOS:
   source .venv/bin/activate
   ```

2. **Install the package in development mode along with all dependencies:**
   ```bash
   pip install -e .
   ```

### Dependencies

The package requires:
- Python 3.10+
- NumPy
- PyBullet
- Matplotlib

These will be automatically installed when you run `pip install -e .`

## How to Run

You can run the program in two ways:

### 1. With Command Line Arguments
Specify the target position and orientation when running the script:
```bash
python main.py --position X Y Z --orientation ROLL PITCH YAW
```

**Example:**
```bash
python main.py --position 5 -1.5 1 --orientation 0 90 -90
```

### 2. Using Default Values
If no arguments are provided, the script will use default values:
- Position: [5.0, -1.5, 1.0] meters
- Orientation: [0, 90, -90] degrees (roll, pitch, yaw)

Simply run:
```bash
python main.py
```

## Output Files

The program generates the following output files:

### 1. Log Files
- **Location:** `log/robot_ik.log`
- **Contents:**
  - Timestamped log messages with different severity levels (INFO, WARNING, ERROR)
  - Information about the IK solving process
  - Details about the best solution found (joint angles and linear position)
  - Iteration data during the simulation (joint positions and linear axis values)

### 2. End-Effector Trajectory Plot
- **File:** `end_effector_trajectory.png`
- **Contents:**
  - A 3D plot showing the path of the end-effector
  - Trajectory line in blue
  - Green circle (○) marking the start of the trajectory
  - Red cross (×) marking the end of the trajectory
  - Green star (✱) showing the initial position (if provided)
  - Red star (✱) showing the target position
  - Grid lines and axis labels for better orientation
  - Legend explaining the different markers

## Project Structure

```
.
├── main.py                 # Main script to run the IK solver and visualization
├── log/                    # Directory containing log files
│   └── robot_ik.log       # Log file with execution details
├── end_effector_trajectory.png  # Generated trajectory plot
├── src/
│   ├── __init__.py
│   ├── ik_solver.py       # IK solver implementation
│   └── visualizer.py      # PyBullet visualization and trajectory plotting
└── robot-urdfs/           # Robot model files
    ├── abb_irb6700_150_320/
    │   └── abb_irb6700_150_320.urdf
    └── linear_axis/
        └── linear_axis.urdf
```

The URDF files in the `robot-urdfs` directory describe the robot's geometry, kinematics, and visual properties.
