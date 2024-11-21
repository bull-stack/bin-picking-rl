# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.assets import RigidObjectCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

# from omni.isaac.lab_tasks.manager_based.manipulation.lift import mdp
# from omni.isaac.lab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
from base_environments import mdp
##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
# from omni.isaac.lab_assets.universal_robots import UR10_CFG  # isort: skip
from base_environments.bin_picking_env_cfg import BinPickingEnvCfg
from .ur10 import UR10_CFG

@configclass
class UR10BinPickingEnvCfg(BinPickingEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        self.scene.robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot") # type: ignore

        # Set actions for the specific robot type (ur10)
        self.actions.arm_action = mdp.JointPositionActionCfg( # type: ignore
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )
        # Set the body name for the end effector
        self.commands.object_pose_1.body_name = "ee_link" # type: ignore
        # self.commands.object_pose_2.body_name = "ee_link" # type: ignore
        # self.commands.object_pose_3.body_name = "ee_link" # type: ignore
        # self.commands.object_pose_4.body_name = "ee_link" # type: ignore
        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0, 0, 0.9], rot=[1, 0, 0, 0]), # type: ignore
            spawn=UsdFileCfg(
                usd_path=f"usd/torus1.usd",
                scale=(1, 1, 1),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
    
        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy() # type: ignore
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ee_link",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034], # type: ignore
                    ),
                ),
            ],
        )


@configclass
class UR10BinPickingEnvCfg_PLAY(UR10BinPickingEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False # type: ignore