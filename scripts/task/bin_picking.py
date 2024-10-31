from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.stage import get_stage_units, add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.physx import get_physx_scene_query_interface
from omni.physx.scripts import utils
from pxr import UsdPhysics
from .bin import Bin
import omni
import numpy as np
import random
import carb
import threading
import time
import math

class BinPicking(BaseTask):
    def __init__(self, name: str):
        BaseTask.__init__(self, name=name)
        self.robot = None
        self.scene_path = "/World/Scene"  
        
        self.bins_pose = np.array([
            [(0, 0, 0), ( 0.4, 0.6, 0.64)],
            [(0, 0, 0), (-0.4, 0.6, 0.64)],
            [(0, 0, np.pi), ( 0.4,-0.6, 0.64)],
            [(0, 0, np.pi), (-0.4,-0.6, 0.64)]
        ])
        self.assets_root_path = get_assets_root_path()
        self.torus_asset_path = self.assets_root_path + "/Isaac/Props/Shapes/torus.usd"

        self.toruses_colors = np.array([
            [0.7, 0.0, 0.0],
            [0.0, 0.0, 0.7],
            [0.7, 0.7, 0.0],
            [0.0, 0.7, 0.0]
        ])
    
        self.toruses = []
        self.toruses_to_add = 0
        self.count = 0
        self.robot_start = False
        self.target_torus = None
        self.target_bin = None
        self.stage = omni.usd.get_context().get_stage()
        self._pause = False

    def random_transform(self):
        x = random.uniform(-0.35, 0.35)
        y = random.uniform(-0.25, 0.25)
        z = 1
        position = np.array([x, y, z])
        return position / get_stage_units()
    
    def find_unique_names(self, inital_name, initial_prim_path):
        name = find_unique_string_name(
                initial_name=inital_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        prim_path = find_unique_string_name(
            initial_name=initial_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        return name, prim_path
    
    
    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        
        table = FixedCuboid(
            prim_path=f'{self.scene_path}/table',
            name="table",
            position=np.array([0, 0, 0.83]),
            scale=np.array([1.15, 0.8, .05]),
            color=np.array([71.0 / 255.0, 165.0 / 255.0, 1.0])
        )
        table.set_world_pose(position=np.array([0, 0, 0.83]))
        mount = FixedCuboid(
            prim_path=f'{self.scene_path}/mount',
            name="mount",
            position=np.array([0, 0.7, 0.415]),
            scale=np.array([.15, .15, .83]),
            color=np.array([71.0 / 255.0, 165.0 / 255.0, 1.0])
        )
        mount.set_world_pose(position=np.array([0, 0.7, 0.415]))
        self.bins = []
        self.bin_sensor = {}
        for i, (bin_orient, bin_pos) in enumerate(self.bins_pose):
            usd_path = f'usd/bin.usd'
            name = f"bin_{i}"
            name, prim_path = self.find_unique_names(name, f"{self.scene_path}/Bins/{name}")
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            prim = XFormPrim(name=name, prim_path=prim_path, position=bin_pos, orientation=euler_angles_to_quat(bin_orient))            
            bin = self.scene.add(prim)
            bin.set_world_pose(position=bin_pos, orientation=euler_angles_to_quat(bin_orient))
            self.bins.append(Bin(prim_path, bin, False))
        # Dictionary to keep track of bin occupancy status
        # self.bin_occupancy = {bin.name: False for bin in self.bins}
        self.scene.add(table)
        self.scene.add(mount)
        self.robot = self.set_robot()
        self.scene.add(self.robot)
        self.unoccupied_bins = []
        

    def has_target(self):
        return self.target_torus is not None and self.target_bin is not None
            
    def get_params(self):
        params_representation = dict()
        if self.has_target():
            params_representation["torus_name"] = {"value": self.target_torus.name, "modifiable": True}
            params_representation["bin_name"] = {"value": self.target_bin.bin_prim.name, "modifiable": True}
        params_representation["robot_name"] = {"value": self.robot.name, "modifiable": False}
        return params_representation

    def check_if_target_in_bin(self):
        bin_position, bin_orientation = self.target_bin.bin_prim.get_world_pose()
        bin_orientation = quat_to_euler_angles(bin_orientation)
        x, y = self.angle_to_vector(bin_orientation[2])
        if y < 0:
            bin_position[1] += 0.19
        else:
            bin_position[1] -= 0.19
            
        origin = carb.Float3(bin_position[0], bin_position[1], bin_position[2] + 0.04)
        rayDir = carb.Float3(x, y, 0.0)
        distance = 0.37
        # physX query to detect closest hit
        hit = get_physx_scene_query_interface().raycast_closest(origin, rayDir, distance)
        return (hit["hit"])
    
    def angle_to_vector(self, angle):
        # Calculate the normal vector components
        x = math.sin(angle)
        y = math.cos(angle)
        return x, y
    
    def target_reached(self, prim_path):
        doors = ['door_left', 'door_right']
        # check of torus is in a bin
        if self.check_if_target_in_bin():
            self.set_bin_occupancy(True)
            self.open_bin(doors, prim_path)
        else:
            self.set_bin_occupancy(False)
    
    def open_bin(self, doors, prim_path):
        """Open the specified bin"""
        for door in doors:
            bin_joint_drive = UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath(f"{prim_path}/{door}/RevoluteJoint"), "angular")
            bin_joint_drive.GetStiffnessAttr().Set(0)
            bin_joint_drive.GetTargetVelocityAttr().Set(200)
            
            # Start a new thread to close the bin after a delay
            self.close_thread = threading.Thread(target=self.close_bin_with_delay, args=(door, prim_path))
            self.close_thread.start()
            
    def close_bin(self, door, prim_path):
        """Close the specified bin"""
        bin_joint_drive = UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath(f"{prim_path}/{door}/RevoluteJoint"), "angular")
        bin_joint_drive.GetTargetVelocityAttr().Set(-200)       

    def close_bin_with_delay(self, door, prim_path):
        time.sleep(1)
        self.close_bin(door, prim_path)
            
    def set_bin_occupancy(self, is_occupied):
        self.target_bin.occupied = is_occupied

        
    # Define a function to check for unoccupied bins
    def check_bins_availability(self):
        while True:
            unoccupied_bins = [bin for bin in self.bins if not bin.occupied]
            if unoccupied_bins:
                self.unoccupied_bins = unoccupied_bins
                break

            carb.log_info("All bins are occupied. Waiting for a bin to become available...")

            # Add a delay before rechecking for unoccupied bins
            time.sleep(1)  # Import time module for the sleep function

    # Modify your method to use asynchronous waiting for unoccupied bins
    def pick_torus_and_target_bin(self):
        # Start the asynchronous task to continuously check for unoccupied bins
        unoccupied_bins_thread = threading.Thread(target=self.check_bins_availability)
        unoccupied_bins_thread.start()
        
        # Wait for the unoccupied bins list to be populated
        while not self.unoccupied_bins:
            time.sleep(0.5)  # Adjust this delay as needed
            
        min_distance = float('inf')
        for bin in self.unoccupied_bins:
            bin_pos, _ = bin.bin_prim.get_world_pose()
            for torus in self.toruses:
                torus_pos, _ = torus.get_world_pose()
                # Pick a bin closest to the torus
                distance = np.linalg.norm(np.array(torus_pos) - np.array(bin_pos))
                if distance < min_distance:
                    min_distance = distance
                    self.target_bin = bin
                    self.target_torus = torus
                    
    def get_observations(self):
        joints_state = self.robot.get_joints_state()
        end_effector_position, end_effector_orientation = self.robot.end_effector.get_world_pose()

        result = {}
        torus_position, torus_orienation = self.target_torus.get_world_pose()
        bin_position, bin_orientation = self.target_bin.bin_prim.get_world_pose()
        result[f"{self.target_torus.name}"] = {
            "position": torus_position,
            "orientation": torus_orienation,
            "target_bin_pos": bin_position,
            "target_bin_orientation": bin_orientation,
        }

        result["my_ur10"] = {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
                "end_effector_orientation": end_effector_orientation,
            }
        return result
    
    
    def add_torus(self):
        name = f"torus_{self.count}"
        name, prim_path = self.find_unique_names(name, f"{self.scene_path}/Torus/{name}")
        add_reference_to_stage(usd_path=self.torus_asset_path, prim_path=prim_path)
        rprim = RigidPrim(name=name, prim_path=prim_path, scale= np.array([0.12, 0.12, 0.12]))

        color = self.toruses_colors[random.randint(0, len(self.toruses_colors) - 1)]
        material = OmniPBR(name=name, prim_path=f"{prim_path}/Visual", color=color)
        rprim.apply_visual_material(material)
        
        name = name.capitalize()
        prim = self.stage.GetPrimAtPath(f'{prim_path}/Torus')
        utils.setCollider(prim, approximationShape="convexDecomposition")
        torus = self.scene.add(rprim)

        self.torus_tranform(torus)
        self.toruses.append(torus)
        self.toruses_to_add -= 1
        self.count += 1

    def torus_tranform(self, object):
        position = self.random_transform()
        object.set_world_pose(position=position)
        object.set_visibility(True)

    def set_num_torus(self, count):
        self.toruses_to_add = count

    def pre_step(self, time_step_index: int, simulation_time: float):
        BaseTask.pre_step(self, time_step_index=time_step_index, simulation_time=simulation_time)
        if self.has_target():
            self.target_reached(self.target_bin.prim_path)
                                    
        if self.toruses_to_add > 0 and time_step_index % 20 == 0:
            self.add_torus()
            
    def start_robot(self):
        if len(self.toruses) >= 0:
            self.robot_start = True
        else:
            carb.log_error("Need to add toruses to simulation")

    def post_reset(self):
        self.toruses_to_add = 0
        self.count = 0
        self.robot_start = False

    def cleanup(self) -> None:
        for i in range(len(self.toruses)):
            self.scene.remove_object(self.toruses[i].name)
        self.toruses = []

    def remove_torus(self):
        for torus in self.toruses:
            torus_pos, _ = torus.get_world_pose()
            if torus_pos[2] <= 0.1:
                self.scene.remove_object(torus.name)
                self.toruses.remove(torus)

                
    def is_done(self):
        # Check if all toruses have been successfully placed in bins
        return self.toruses_to_add <= 0 and len(self.toruses) == 0
    
    def clear_targets(self):
        self.remove_torus()    
        self.target_bin = None
        self.target_torus = None
