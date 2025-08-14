from ros_gym_env.envs.drl_vo_env import DRLVOEnv
from ros_gym_env.envs.crowd_height_env import CrowdHeightEnv
from ros_gym_env.envs.my_env import MyEnv
from ros_gym_env.envs.closest_obstacles import ClosestObstaclesEnv
from ros_gym_env.envs.hbsn_env import HBSNEnv


class Env():
    def __init__(self, env):
        self.env = env

env_factory = dict()
env_factory['myenv'] = MyEnv
env_factory['closest_obstacles'] = ClosestObstaclesEnv
env_factory['drl-vo'] = DRLVOEnv
env_factory['crowd_height'] = CrowdHeightEnv
env_factory['hbsn'] = HBSNEnv