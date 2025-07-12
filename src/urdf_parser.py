import numpy as np
import xml.etree.ElementTree as ET
from typing import List, Tuple, Dict
import math

class URDFParser:
    """Parse URDF files to extract robot parameters."""
    
    def __init__(self, urdf_content: str):
        self.root = ET.fromstring(urdf_content)
        self.joints = {}
        self.links = {}
        self.robot_name = self.root.get('name', 'Unknown')
        self._parse_urdf()
    
    def _parse_urdf(self):
        """Parse URDF content and extract joint and link information."""
        # Parse joints
        for joint in self.root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            
            # Get origin
            origin = joint.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
                rpy = origin.get('rpy', '0 0 0').split()
                xyz = [float(x) for x in xyz]
                rpy = [float(x) for x in rpy]
            else:
                xyz = [0, 0, 0]
                rpy = [0, 0, 0]
            
            # Get axis
            axis = joint.find('axis')
            if axis is not None:
                axis_xyz = axis.get('xyz', '0 0 1').split()
                axis_xyz = [float(x) for x in axis_xyz]
            else:
                axis_xyz = [0, 0, 1]
            
            # Get limits
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.get('lower', '0'))
                upper = float(limit.get('upper', '0'))
                velocity = float(limit.get('velocity', '0'))
                effort = float(limit.get('effort', '0'))
            else:
                lower = -math.pi
                upper = math.pi
                velocity = 1.0
                effort = 100.0
            
            # Get parent and child links
            parent = joint.find('parent')
            child = joint.find('child')
            
            if parent is not None and child is not None:
                self.joints[joint_name] = {
                    'type': joint_type,
                    'origin_xyz': xyz,
                    'origin_rpy': rpy,
                    'axis': axis_xyz,
                    'limits': (lower, upper, velocity, effort),
                    'parent': parent.get('link'),
                    'child': child.get('link')
                }
        
        # Parse links
        for link in self.root.findall('link'):
            link_name = link.get('name')
            self.links[link_name] = {'visual': None, 'collision': None}
            
            # Parse visual elements
            visual = link.find('visual')
            if visual is not None:
                origin = visual.find('origin')
                if origin is not None:
                    xyz = origin.get('xyz', '0 0 0').split()
                    rpy = origin.get('rpy', '0 0 0').split()
                    xyz = [float(x) for x in xyz]
                    rpy = [float(x) for x in rpy]
                else:
                    xyz = [0, 0, 0]
                    rpy = [0, 0, 0]
                
                geometry = visual.find('geometry')
                if geometry is not None:
                    box = geometry.find('box')
                    if box is not None:
                        size = [float(x) for x in box.get('size', '0.1 0.1 0.1').split()]
                        self.links[link_name]['visual'] = {
                            'type': 'box',
                            'size': size,
                            'origin_xyz': xyz,
                            'origin_rpy': rpy
                        }
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """Get joint limits as a list of (lower, upper) tuples."""
        limits = []
        for joint in self.joints.values():
            limits.append((joint['limits'][0], joint['limits'][1]))
        return limits
    
    def get_link_origins(self) -> Dict[str, np.ndarray]:
        """Get the origin of each link in the robot's base frame."""
        origins = {}
        origins['base_link'] = np.zeros(3)
        
        # Simple forward kinematics to get link positions
        for joint_name, joint in self.joints.items():
            parent = joint['parent']
            child = joint['child']
            
            # Get parent's origin
            if parent in origins:
                parent_origin = origins[parent]
                
                # Simple approximation: just add the origin xyz
                origin = np.array(joint['origin_xyz'])
                origins[child] = parent_origin + origin
        
        return origins