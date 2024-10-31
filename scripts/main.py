"""Launch Isaac Sim Simulator first."""
import argparse
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac Sim Simulator for Robot Bin Picking using RL")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
from dataclasses import MISSING
from gui import tkinter_gui

from omni.isaac.lab.sim import SimulationContext, SimulationCfg
from omni.isaac.lab.assets import AssetBaseCfg
from omni.isaac.lab.scene import InteractiveSceneCfg, InteractiveScene
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ActionTermCfg as ActionTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.managers import SceneEntityCfg 
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from omni.isaac.lab_assets import UR10_CFG  # isort:skip
import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.envs.mdp as mdp  # isort:skip
import numpy as np
import carb
import torch
import math

@configclass
class ActionCfg:
    arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )
    arm_action: ActionTerm = MISSING # type: ignore
    gripper_action: ActionTerm | None = None

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)
        
@configclass
class UR10BinSceneCfg(InteractiveSceneCfg):

    # terrain - flat terrain plane
    ground = AssetBaseCfg(
        prim_path="/World/ground", 
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )
       
    table = AssetBaseCfg(
        prim_path="/World/Table",
        spawn=sim_utils.CuboidCfg(
            size=(1.15, 0.8, .05), 
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color= (71.0 / 255.0, 165.0 / 255.0, 1.0), metallic=0.5),
            ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 0.83)),
    )
    
    mount = AssetBaseCfg(
        prim_path="/World/Mount",
        spawn=sim_utils.CuboidCfg(
            size=(.15, .15, .83),
            mass_props=sim_utils.MassPropertiesCfg(mass=5.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color= (71.0 / 255.0, 165.0 / 255.0, 1.0), metallic=0.5),      
            ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0.7, 0.415)),
    )
    
    # for (count, bin_orient, bin_pos) in ([
    #         [1, (0, 0, 0), ( 0.4, 0.6, 0.64)],
    #         [2, (0, 0, 0), (-0.4, 0.6, 0.64)],
    #         [3, (0, 0, np.pi), ( 0.4,-0.6, 0.64)],
    #         [4, (0, 0, np.pi), (-0.4,-0.6, 0.64)]
    #     ]):
    #     pass
    bin_1 = AssetBaseCfg(
        prim_path=f"/World/Objects/bin_1",
        spawn=sim_utils.UsdFileCfg(usd_path='bin/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.4, 0.6, 0.64)),
    )
    bin_2 = AssetBaseCfg(
        prim_path=f"/World/Objects/bin_2",
        spawn=sim_utils.UsdFileCfg(usd_path='bin/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.4, 0.6, 0.64)),
    )
    bin_3 = AssetBaseCfg(
        prim_path=f"/World/Objects/bin_3",
        spawn=sim_utils.UsdFileCfg(usd_path='bin/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.4,-0.6, 0.64)),
    )
    bin_4 = AssetBaseCfg(
        prim_path=f"/World/Objects/bin_4",
        spawn=sim_utils.UsdFileCfg(usd_path='bin/bin.usd'),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.4,-0.6, 0.64)),
    )

    robot: ArticulationCfg = MISSING # type: ignore
    
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )
    # robot = UR10_CFG.replace( # type: ignore
    #     prim_path="{ENV_REGEX_NS}/Robot",
    #     init_state=ArticulationCfg.InitialStateCfg(
    #         pos=(0, 0.7, 0.83), 
    #         rot=(0.7071, 0.0000, 0.0000, -0.7071)
    #         # joint_pos={
    #         # "shoulder_pan_joint": -np.pi / 2,
    #         # "shoulder_lift_joint": -np.pi / 2,
    #         # "elbow_joint": -np.pi / 2,
    #         # "wrist_1_joint": -np.pi / 2,
    #         # "wrist_2_joint": np.pi / 2,
    #         # "wrist_3_joint": 0.0
    #         # }
    #     )
    # ) 


def load_scene():
    pass

def start_robot():
    pass
def reset():
    pass
def set_num_objects(count):
        toruses_to_add = count    
        
def run_simulator(sim, scene, window):
    robot = scene["robot"]  # type: ArticulationCfg
    sim_dt = sim.get_physics_dt()
    count = 0
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            root_state = robot.data.default_root_state.clone() # type: ignore
            root_state[:, :3] += scene.env_origins
            robot.write_root_state_to_sim(root_state) # type: ignore
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone() # type: ignore
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel) # type: ignore
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")
        # Apply random action
        # -- generate random joint efforts
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0 # type: ignore
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts) # type: ignore
        # -- write data to sim
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)
    
    
def main():
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2, 2, 2], [0, 0, 0]) # type: ignore
    
    scene_cfg = UR10BinSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    tk_window = tkinter_gui.TkinterGui(set_num_objects, start_robot, reset)
    tk_window.create_frame()
    tk_window.create_sim_frame()  
    
    sim.reset()
    run_simulator(sim, scene, tk_window)
    
    
if __name__ == "__main__":
    # get simulation context
    main()
    simulation_app.close()