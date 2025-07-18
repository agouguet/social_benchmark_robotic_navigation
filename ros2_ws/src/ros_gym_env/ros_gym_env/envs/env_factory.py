from ros_gym_env.envs.ros_gym_env import RosGymEnv
from ros_gym_env.envs.simple_env import RosSimpleEnv
from ros_gym_env.envs.mlagent_gym_env import MlAgentGymEnv


class Env():
    def __init__(self, env):
        self.env = env

env_factory = dict()
env_factory['ros_gym_env'] = RosGymEnv
env_factory['simple_env'] = RosSimpleEnv
env_factory['mlagent_env'] = MlAgentGymEnv