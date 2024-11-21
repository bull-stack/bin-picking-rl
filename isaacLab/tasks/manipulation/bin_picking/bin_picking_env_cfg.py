# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.markers import VisualizationMarkers, VisualizationMarkersCfg
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from omni.isaac.lab.sensors.frame_transformer import OffsetCfg
import mdp
##
# Scene definition
##
FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy() # type: ignore
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)

@configclass
class BinPickingSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING # type: ignore
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING # type: ignore
    # target object: will be populated by agent env cfg
    object: RigidObjectCfg = MISSING # type: ignore
    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(1.15, 0.8, .05), 
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color= (71.0 / 255.0, 165.0 / 255.0, 1.0), metallic=0.5),
            ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 0.83)),
    )

    mount = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Mount",
        spawn=sim_utils.CuboidCfg(
            size=(.15, .15, .83),
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color= (71.0 / 255.0, 165.0 / 255.0, 1.0), metallic=0.5),      
            ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0.7, 0.415)),
    )
    bin_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/bin_1",
        spawn=sim_utils.UsdFileCfg(usd_path='usd/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.4, 0.6, 0.64)),

    )
    bin_2 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/bin_2",
        spawn=sim_utils.UsdFileCfg(usd_path='usd/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.4, 0.6, 0.64)),
    )
    bin_3 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/bin_3",
        spawn=sim_utils.UsdFileCfg(usd_path='usd/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.4,-0.6, 0.64)),
    )
    bin_4 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/bin_4",
        spawn=sim_utils.UsdFileCfg(usd_path='usd/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.4,-0.6, 0.64)),
    )
    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0]), # type: ignore
        spawn=GroundPlaneCfg(),
    )

    # bin_1_frame = FrameTransformerCfg(
    #     prim_path="{ENV_REGEX_NS}/bin_1/sektion",
    #     debug_vis=True,
    #     visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/bin1FrameTransformer"),
    #     target_frames=[
    #         FrameTransformerCfg.FrameCfg(
    #             prim_path="{ENV_REGEX_NS}/bin_1/bin_1_frame",
    #             name="bin_1_frame",
    #             offset=OffsetCfg(
    #                 pos=(0, 0, 0),
    #             ),
    #         ),
    #     ],
    # )
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""
    
    object_pose_1 = mdp.UniformPoseCommandCfg( # type: ignore
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg # type: ignore
        resampling_time_range=(5.0, 5.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges( # type: ignore
            pos_x=(0.1, 0.1), pos_y=(0.4, 0.4), pos_z=(0.05, 0.05), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
        ),
    )
    # object_pose_2 = mdp.UniformPoseCommandCfg( # type: ignore
    #     asset_name="robot",
    #     body_name=MISSING,  # will be set by agent env cfg # type: ignore
    #     resampling_time_range=(5.0, 5.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges( # type: ignore
    #         pos_x=(0.2, 0.8), pos_y=(-0.25, 0.25), pos_z=(1, 1), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
    #     ),
    # )
    # object_pose_3 = mdp.UniformPoseCommandCfg( # type: ignore
    #     asset_name="robot",
    #     body_name=MISSING,  # will be set by agent env cfg # type: ignore
    #     resampling_time_range=(5.0, 5.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges( # type: ignore
    #         pos_x=(0.2, 0.8), pos_y=(-0.25, 0.25), pos_z=(1, 1), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
    #     ),
    # )
    # object_pose_4 = mdp.UniformPoseCommandCfg( # type: ignore
    #     asset_name="robot",
    #     body_name=MISSING,  # will be set by agent env cfg # type: ignore
    #     resampling_time_range=(5.0, 5.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges( # type: ignore
    #         pos_x=(0.2, 0.8), pos_y=(-0.25, 0.25), pos_z=(1, 1), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
    #     ),        
    # )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg = MISSING # type: ignore


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel) # type: ignore
        joint_vel = ObsTerm(func=mdp.joint_vel_rel) # type: ignore
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose_1"}) # type: ignore
        actions = ObsTerm(func=mdp.last_action) # type: ignore

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset") # type: ignore

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform, # type: ignore
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="Torus"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)

    dragging_object = RewTerm(func=mdp.object_is_dragged, params={"minimal_drag": 0.04}, weight=15.0)

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_drag": 0.04, "command_name": "object_pose_1"},
        weight=16.0,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.05, "minimal_drag": 0.04, "command_name": "object_pose_1"},
        weight=5.0,
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4) # type: ignore

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2, # type: ignore
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True) # type: ignore

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("object")} # type: ignore
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000} # type: ignore
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000} # type: ignore
    )


##
# Environment configuration
##


@configclass
class BinPickingEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""

    # Scene settings
    scene: BinPickingSceneCfg = BinPickingSceneCfg(num_envs=4096, env_spacing=2.5) # type: ignore
    # # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625