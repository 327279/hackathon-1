---
sidebar_position: 2
---

# Chapter 1: Isaac Sim Deep Dive

Isaac Sim is the most advanced robot simulator available today. Built on NVIDIA Omniverse, it provides photorealistic rendering, accurate physics, and seamless integration with AI training pipelines. This chapter covers everything you need to master Isaac Sim.

## Understanding USD (Universal Scene Description)

Isaac Sim uses **USD** (developed by Pixar) as its native scene format. Understanding USD is essential for working effectively with Isaac Sim.

### USD Concepts

| Concept | Description | Example |
|---------|-------------|---------|
| **Stage** | The complete scene container | Your simulation world |
| **Prim** | Any object in the scene | Robot, sensor, light |
| **Schema** | Type definition for prims | RigidBody, Articulation |
| **Property** | Attributes of a prim | Position, mass, color |
| **Layer** | Composable scene parts | Robot layer + environment layer |

### USD Structure

```python
# USD hierarchy example
/World
├── /GroundPlane
├── /Lighting
│   ├── /Sun
│   └── /AmbientLight
├── /Environment
│   ├── /Table
│   └── /Shelf
└── /Robot
    ├── /base_link
    ├── /joint_1
    └── /end_effector
```

### Working with USD in Python

```python
from pxr import Usd, UsdGeom, UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage

# Get the current USD stage
stage = get_current_stage()

# Create a new prim
xform = UsdGeom.Xform.Define(stage, "/World/MyObject")

# Set transform
xform.AddTranslateOp().Set((1.0, 2.0, 0.5))
xform.AddRotateXYZOp().Set((0, 0, 45))

# Add physics
rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
rigid_body_api.CreateRigidBodyEnabledAttr(True)
```

## Importing Robots

### From URDF

```python
from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.stage import get_current_stage

# Configure URDF importer
urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.import_inertia_tensor = True
import_config.distance_scale = 1.0
import_config.density = 1000.0
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 1000.0
import_config.default_position_drive_damping = 100.0

# Import robot
robot_prim_path = urdf_interface.parse_urdf(
    "/path/to/robot.urdf",
    import_config
)
```

### From MJCF (MuJoCo)

```python
from omni.importer.mjcf import _mjcf

mjcf_interface = _mjcf.acquire_mjcf_interface()
import_config = _mjcf.ImportConfig()
import_config.fix_base = False
import_config.import_sites = True

robot_prim = mjcf_interface.import_asset(
    "/path/to/robot.xml",
    import_config
)
```

## Creating Environments

### Procedural Environment Generation

```python
from omni.isaac.core.prims import XFormPrim, GeometryPrim
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
import numpy as np

class WarehouseEnvironment:
    def __init__(self, world):
        self.world = world
        self.objects = []
        
    def create_ground(self):
        self.world.scene.add_default_ground_plane(
            z_position=0,
            name="ground",
            static_friction=0.5,
            dynamic_friction=0.5
        )
    
    def create_shelves(self, num_rows=3, num_cols=4):
        """Create a grid of shelving units."""
        shelf_spacing_x = 3.0
        shelf_spacing_y = 2.0
        
        for row in range(num_rows):
            for col in range(num_cols):
                x = col * shelf_spacing_x
                y = row * shelf_spacing_y
                
                shelf = FixedCuboid(
                    prim_path=f"/World/Shelves/shelf_{row}_{col}",
                    name=f"shelf_{row}_{col}",
                    position=np.array([x, y, 1.0]),
                    scale=np.array([0.5, 2.0, 2.0]),
                    color=np.array([0.4, 0.3, 0.2])
                )
                self.world.scene.add(shelf)
                self.objects.append(shelf)
    
    def spawn_objects(self, num_objects=20):
        """Spawn graspable objects on shelves."""
        for i in range(num_objects):
            # Random position on a shelf
            x = np.random.uniform(0, 9)
            y = np.random.uniform(0, 4)
            z = np.random.uniform(0.5, 1.8)
            
            # Random size
            size = np.random.uniform(0.05, 0.15)
            
            # Random color
            color = np.random.rand(3)
            
            obj = DynamicCuboid(
                prim_path=f"/World/Objects/obj_{i}",
                name=f"obj_{i}",
                position=np.array([x, y, z]),
                scale=np.array([size, size, size]),
                color=color,
                mass=0.1
            )
            self.world.scene.add(obj)
            self.objects.append(obj)
```

### Lighting Setup

```python
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdLux

def setup_lighting(stage):
    """Configure realistic lighting for the scene."""
    
    # Dome light for ambient illumination
    dome_light = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome_light.CreateIntensityAttr(1000)
    dome_light.CreateTextureFileAttr("path/to/hdri.hdr")
    
    # Directional light (sun)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lighting/Sun")
    sun.CreateIntensityAttr(2500)
    sun.CreateAngleAttr(0.53)  # Angular size of the sun
    sun.CreateColorAttr((1.0, 0.95, 0.9))  # Warm sunlight
    
    # Position the sun
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 30, 0))
    
    # Area lights for indoor scenes
    area_light = UsdLux.RectLight.Define(stage, "/World/Lighting/Ceiling")
    area_light.CreateIntensityAttr(5000)
    area_light.CreateWidthAttr(2.0)
    area_light.CreateHeightAttr(2.0)
```

## Sensor Configuration

### Camera Configuration with Ray Tracing

```python
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.render_product import create_hydra_texture

class RGBDCamera:
    def __init__(self, world, prim_path, resolution=(1280, 720)):
        self.world = world
        self.prim_path = prim_path
        self.resolution = resolution
        
        # Create camera prim
        self.camera = Camera(
            prim_path=prim_path,
            frequency=30,
            resolution=resolution
        )
        
    def initialize(self):
        self.camera.initialize()
        
        # Enable additional render products
        self.camera.add_distance_to_camera_to_frame()
        self.camera.add_normals_to_frame()
        self.camera.add_instance_segmentation_to_frame()
        
    def get_frame(self):
        """Get all sensor data from camera."""
        return {
            'rgb': self.camera.get_rgb(),
            'depth': self.camera.get_depth(),
            'normals': self.camera.get_normals(),
            'segmentation': self.camera.get_instance_segmentation()
        }
    
    def set_intrinsics(self, fx, fy, cx, cy):
        """Set camera intrinsic parameters."""
        self.camera.set_focal_length(fx / self.resolution[0] * 36)  # Convert to mm
        self.camera.set_horizontal_aperture(36.0)  # Standard 35mm format
```

### LiDAR Configuration

```python
from omni.isaac.range_sensor import _range_sensor

class SimulatedLiDAR:
    def __init__(self, world, prim_path):
        self.lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        self.prim_path = prim_path
        
    def configure(self):
        """Configure LiDAR sensor parameters."""
        # Get the lidar prim
        result, lidar = self.lidar_interface.get_lidar_sensor_interface(self.prim_path)
        
        if result:
            lidar.set_rotation_frequency(10.0)  # 10 Hz rotation
            lidar.set_min_range(0.1)
            lidar.set_max_range(100.0)
            lidar.set_horizontal_resolution(0.2)  # 0.2 degree resolution
            lidar.set_vertical_fov_up(15.0)
            lidar.set_vertical_fov_down(-15.0)
            lidar.set_num_channels(32)
    
    def get_point_cloud(self):
        """Get point cloud data."""
        return self.lidar_interface.get_point_cloud_data(self.prim_path)
    
    def get_linear_depth(self):
        """Get linear depth buffer."""
        return self.lidar_interface.get_linear_depth_data(self.prim_path)
```

## Physics Configuration

### Articulation (Robot) Physics

```python
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

class RobotController:
    def __init__(self, world, prim_path):
        self.robot = world.scene.add(
            Articulation(
                prim_path=prim_path,
                name="robot"
            )
        )
        
    def initialize(self):
        self.robot.initialize()
        self.num_dof = self.robot.num_dof
        
        # Get joint limits
        self.joint_limits = self.robot.get_articulation_controller().get_joint_limits()
        
        print(f"Robot has {self.num_dof} DOF")
        print(f"Joint names: {self.robot.dof_names}")
        
    def set_joint_positions(self, positions):
        """Set target joint positions."""
        action = ArticulationAction(
            joint_positions=np.array(positions)
        )
        self.robot.apply_action(action)
    
    def set_joint_velocities(self, velocities):
        """Set target joint velocities."""
        action = ArticulationAction(
            joint_velocities=np.array(velocities)
        )
        self.robot.apply_action(action)
    
    def set_joint_efforts(self, efforts):
        """Apply joint torques/forces."""
        action = ArticulationAction(
            joint_efforts=np.array(efforts)
        )
        self.robot.apply_action(action)
    
    def get_state(self):
        """Get full robot state."""
        return {
            'joint_positions': self.robot.get_joint_positions(),
            'joint_velocities': self.robot.get_joint_velocities(),
            'body_position': self.robot.get_world_pose()[0],
            'body_orientation': self.robot.get_world_pose()[1]
        }
```

### Contact Reporting

```python
from omni.isaac.core.utils.physics import get_contact_data

def get_gripper_contacts(robot_prim_path, gripper_link_name):
    """Get contact forces on gripper."""
    gripper_path = f"{robot_prim_path}/{gripper_link_name}"
    
    contacts = get_contact_data(gripper_path)
    
    total_force = np.zeros(3)
    contact_points = []
    
    for contact in contacts:
        total_force += contact.force
        contact_points.append(contact.position)
    
    return {
        'force': total_force,
        'points': contact_points,
        'num_contacts': len(contacts)
    }
```

## Synthetic Data Generation

### Domain Randomization

```python
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdShade
import random

class DomainRandomizer:
    def __init__(self, stage):
        self.stage = stage
        
    def randomize_lighting(self):
        """Randomize lighting conditions."""
        sun = get_prim_at_path("/World/Lighting/Sun")
        if sun:
            # Randomize intensity
            sun.GetAttribute("intensity").Set(
                random.uniform(1000, 5000)
            )
            # Randomize color temperature
            r = random.uniform(0.9, 1.0)
            g = random.uniform(0.8, 0.95)
            b = random.uniform(0.7, 0.9)
            sun.GetAttribute("color").Set((r, g, b))
    
    def randomize_textures(self, prim_paths):
        """Randomize material textures."""
        textures = [
            "omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak.mdl",
            "omniverse://localhost/NVIDIA/Materials/Base/Metal/Steel.mdl",
            "omniverse://localhost/NVIDIA/Materials/Base/Stone/Granite.mdl",
        ]
        
        for path in prim_paths:
            prim = get_prim_at_path(path)
            if prim:
                material = UsdShade.Material.Get(self.stage, random.choice(textures))
                UsdShade.MaterialBindingAPI(prim).Bind(material)
    
    def randomize_object_poses(self, object_paths, bounds):
        """Randomize object positions within bounds."""
        for path in object_paths:
            prim = get_prim_at_path(path)
            if prim:
                x = random.uniform(bounds['x_min'], bounds['x_max'])
                y = random.uniform(bounds['y_min'], bounds['y_max'])
                z = random.uniform(bounds['z_min'], bounds['z_max'])
                
                xform = UsdGeom.Xformable(prim)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set((x, y, z))
```

## Complete Simulation Example

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

def run_simulation():
    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Load humanoid robot
    robot = world.scene.add(
        Robot(
            prim_path="/World/Humanoid",
            name="humanoid",
            usd_path="omniverse://localhost/NVIDIA/Robots/Humanoid/humanoid.usd",
            position=np.array([0, 0, 1.05])
        )
    )
    
    # Add camera
    from omni.isaac.sensor import Camera
    camera = Camera(
        prim_path="/World/Humanoid/head/camera",
        frequency=30,
        resolution=(640, 480)
    )
    
    # Reset world
    world.reset()
    robot.initialize()
    camera.initialize()
    
    # Simulation loop
    step = 0
    while True:
        # Get observations
        rgb = camera.get_rgb()
        joint_pos = robot.get_joint_positions()
        
        # Compute actions (your policy here)
        actions = compute_policy(rgb, joint_pos)
        
        # Apply actions
        robot.apply_action(ArticulationAction(joint_positions=actions))
        
        # Step simulation
        world.step(render=True)
        step += 1
        
        if step % 1000 == 0:
            print(f"Step {step}")

if __name__ == "__main__":
    run_simulation()
```

## Key Takeaways

1. **USD** is the foundation - understand the scene format
2. **Multiple import options** - URDF, MJCF, native USD
3. **Photorealistic sensors** - RTX-accelerated cameras and LiDAR
4. **Domain randomization** - Essential for sim-to-real transfer
5. **Python API** - Full control over every aspect of simulation

---

**Next:** [Chapter 2: Isaac ROS Integration](./isaac-ros) or continue to [Module 4: VLA & Humanoids](../module-4-vla/) →
