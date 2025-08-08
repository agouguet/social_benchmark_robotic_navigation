from ros_gym_env.envs.ros_gym_env import RosGymEnv
from ros_gym_env.envs.mlagent_gym_env import MlAgentGymEnv
from ros_gym_env.envs.mlagent_gym_env_test import MlAgentGymEnvTest
from ros_gym_env.envs.drl_vo_env import DRLVOEnv
from ros_gym_env.envs.crowd_height_env import CrowdHeightEnv
from ros_gym_env.envs.my_env import MyEnv


class Env():
    def __init__(self, env):
        self.env = env

env_factory = dict()
env_factory['ros_gym_env'] = RosGymEnv
env_factory['mlagent_env'] = MlAgentGymEnv
env_factory['test'] = MlAgentGymEnvTest
env_factory['myenv'] = MyEnv
env_factory['drl-vo'] = DRLVOEnv
env_factory['crowd_height'] = CrowdHeightEnv