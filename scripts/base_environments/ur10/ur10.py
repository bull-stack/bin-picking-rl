import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

#{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur10/ur10_short_suction.usd
UR10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur10/ur10_short_suction.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=7, solver_velocity_iteration_count=0
        ),
        # activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0, 0.7, 0.83),
        rot=(0.7071, 0.0000, 0.0000, -0.7071),
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)


UR10_HIGH_PD_CFG = UR10_CFG.copy() # type: ignore
UR10_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
# UR10_HIGH_PD_CFG.actuators[".*"].stiffness = 400.0
# UR10_HIGH_PD_CFG.actuators[".*"].damping = 80.0