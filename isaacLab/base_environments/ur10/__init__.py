from . import ik_abs_env_cfg, agents
import gymnasium as gym
##
# Register Gym environments.
##
##
# Inverse Kinematics - Absolute Pose Control
##
print('Isaac-Bin-Picking-UR10-IK-Abs-v0')
gym.register(
    id="Isaac-Bin-Picking-UR10-IK-Abs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.UR10BinPickingEnvCfg_PLAY,
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
    disable_env_checker=True,
)