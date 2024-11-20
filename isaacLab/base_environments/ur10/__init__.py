from . import ik_abs_env_cfg, agents
import gymnasium as gym
##
# Register Gym environments.
##
##
# Inverse Kinematics - Absolute Pose Control
##
gym.register(
    id="Isaac-Bin-Picking-UR10-IK-Abs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.UR10BinPickingEnvCfg_PLAY,
    },
    disable_env_checker=True,
)