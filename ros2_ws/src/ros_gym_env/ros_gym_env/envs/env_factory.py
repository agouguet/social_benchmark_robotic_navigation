from ros_gym_env.envs.ros_gym_env import RosGymEnv
from ros_gym_env.envs.simple_env import RosSimpleEnv
from ros_gym_env.envs.mlagent_gym_env import MlAgentGymEnv
from ros_gym_env.envs.mlagent_gym_env_test import MlAgentGymEnvTest
from ros_gym_env.envs.drl_vo_env import DRLVOEnv


class Env():
    def __init__(self, env):
        self.env = env

env_factory = dict()
env_factory['ros_gym_env'] = RosGymEnv
env_factory['simple_env'] = RosSimpleEnv
env_factory['mlagent_env'] = MlAgentGymEnv
env_factory['test'] = MlAgentGymEnvTest
env_factory['drl-vo'] = DRLVOEnv