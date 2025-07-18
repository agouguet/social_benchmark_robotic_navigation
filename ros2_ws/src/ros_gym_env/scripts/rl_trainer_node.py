#!/usr/bin/env python3

import numpy as np
import os, yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.utils import get_schedule_fn
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback

# from ros_gym_env.ros_gym_env.envs.ros_unity_gym_env_old import RosUnityEnvOld
from ros_gym_env.envs.env_factory import *
from ros_gym_env.envs.env_wrappers import *
from ros_gym_env.policy.policy_factory import *



class Config:
    def __init__(self, d):
        for k, v in d.items():
            if isinstance(v, dict):
                v = Config(v)
            setattr(self, k, v)

def load_ros2_package_config(package_name, relative_path):
    pkg_path = get_package_share_directory(package_name)
    config_path = os.path.join(pkg_path, relative_path)
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)
    return Config(data)


class RLTrainerNode(Node):
    def __init__(self):
        super().__init__('rl_trainer_node')

        self.config_name = "training_002.yaml"
        self.config = load_ros2_package_config("ros_gym_env", "config/"+self.config_name)
        model_name = "model/"
        self.model_path = os.path.join(get_package_share_directory("ros_gym_env"), model_name)
        self.use_model = False

        # Env creation
        env_ids = np.arange(self.config.learning.num_envs)
        env_fns = [self.make_env(i) for i in env_ids]
        self.env = DummyVecEnv(env_fns)
        self.env = VecNormalize(self.env, norm_obs=self.config.env.normalize, norm_reward=self.config.env.normalize)
        self.env = VecEnvDelayWrapper(self.env, delay_sec=(0.05 / self.config.learning.speed_time))

        policy, policy_kwargs = method_factory[self.config.policy.name].get_policy(self.config)

        common_kwargs = {
            'env': self.env,
            'tensorboard_log': self.config.log.log_dir,
            'verbose': 2,
            'n_epochs': self.config.ppo.n_epochs,
            'n_steps': self.config.learning.n_steps,
            'batch_size': self.config.ppo.batch_size,
            'learning_rate': get_schedule_fn(self.config.learning.learning_rate),
            'clip_range': get_schedule_fn(self.config.ppo.clip_range),
            'ent_coef': self.config.ppo.ent_coef,
            'vf_coef': self.config.ppo.vf_coef
        }

        if os.path.exists(self.model_path + ".zip"):
            self.model = PPO.load(self.model_path, **common_kwargs)
            self.use_model = True
        else:
            self.model = PPO(
                policy,
                policy_kwargs=policy_kwargs,
                **common_kwargs,
                normalize_advantage=True,
                seed=self.config.ppo.seed
            )

    def make_env(self, env_id):
        def _init():
            env = env_factory[self.config.env.name](env_id, self.config, env_id_display_log=0)
            env = Monitor(env, os.path.join(self.config.log.log_dir, str(env_id)))
            return env
        return _init

    def start_training(self):

        

        self.get_logger().info("Start training ...")
        log_interval = max(1, int(self.config.log.log_interval / self.config.learning.num_envs))
        postfix = "_norm" if self.config.env.normalize else ""
        name_log = f'{self.config.policy.name}_{self.config.learning.num_envs}_{self.config.env.name}{postfix}'
        
        checkpoint_callback = CheckpointCallback(
            save_freq=self.config.learning.save_model_frequency // self.config.learning.num_envs,  # fréquence = 1M steps globaux
            save_path="./model/",
            name_prefix=name_log+"_checkpoint"
        )
        
        self.model.learn(
            total_timesteps=self.config.learning.total_timesteps,
            log_interval=log_interval,
            tb_log_name=name_log,
            reset_num_timesteps=not self.use_model,
            callback=checkpoint_callback 
        )
        self.model.save("drl_vo_model")
        self.env.close()


def main(args=None):
    rclpy.init(args=args)
    node = RLTrainerNode()

    try:
        node.start_training()
    except Exception as e:
        node.get_logger().error(f"Exception capturée : {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    