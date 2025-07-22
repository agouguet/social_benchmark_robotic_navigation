#!/usr/bin/env python3

import numpy as np
import os, yaml, glob
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from stable_baselines3 import PPO, SAC, DDPG
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.utils import get_schedule_fn
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback

# from ros_gym_env.ros_gym_env.envs.ros_unity_gym_env_old import RosUnityEnvOld
from ros_gym_env.envs.env_factory import *
from ros_gym_env.envs.env_wrappers import *
from ros_gym_env.policy.policy_factory import *
from ros_gym_env.learning.curriculum import CurriculumManager, CurriculumCallback



class Config:
    def __init__(self, d):
        for k, v in d.items():
            if isinstance(v, dict):
                v = Config(v)
            setattr(self, k, v)

    def to_dict(self):
        result = {}
        for k, v in self.__dict__.items():
            if isinstance(v, Config):
                result[k] = v.to_dict()
            else:
                result[k] = v
        return result


def get_latest_log_dir(base_dir, name_log):
        """
        Retourne le dernier dossier créé correspondant à base_dir/name_log*
        """
        pattern = os.path.join(base_dir, f"{name_log}*")
        dirs = [d for d in glob.glob(pattern) if os.path.isdir(d)]
        if not dirs:
            return os.path.join(base_dir, name_log)  # dossier de base
        latest_dir = max(dirs, key=os.path.getmtime)
        return latest_dir


def load_ros2_package_config(package_name, relative_path):
    pkg_path = get_package_share_directory(package_name)
    config_path = os.path.join(pkg_path, relative_path)
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)
    return Config(data)


def get_rl_algo(name):
    name = name.lower()
    algos = {
        "ppo": PPO,
        "sac": SAC,
        "ddpg": DDPG
    }
    if name not in algos:
        raise ValueError(f"RL Algo '{name}' non reconnue. Choisis parmi : {list(algos.keys())}")
    return algos[name]

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
        self.env = VecNormalize(self.env, norm_obs=self.config.env.normalize_observation, norm_reward=self.config.env.normalize_reward)
        self.env = VecEnvDelayWrapper(self.env, delay_sec=(0.05 / self.config.learning.speed_time))

        policy, policy_kwargs = method_factory[self.config.policy.name].get_policy(self.config)

        common_kwargs = {
            'env': self.env,
            'tensorboard_log': self.config.log.log_dir,
            'verbose': 2,
            'n_epochs': self.config.algo.n_epochs,
            'n_steps': self.config.learning.n_steps,
            'batch_size': self.config.algo.batch_size,
            'learning_rate': get_schedule_fn(self.config.learning.learning_rate),
            'clip_range': get_schedule_fn(self.config.algo.clip_range),
            'ent_coef': self.config.algo.ent_coef,
            'vf_coef': self.config.algo.vf_coef
        }

        rl_algo = get_rl_algo(self.config.algo.name)

        if os.path.exists(self.model_path + ".zip"):
            self.model = rl_algo.load(self.model_path, **common_kwargs)
            self.use_model = True
        else:
            self.model = rl_algo(
                policy,
                policy_kwargs=policy_kwargs,
                **common_kwargs,
                normalize_advantage=True,
                seed=self.config.algo.seed
            )
        
        self.curriculum = CurriculumManager(
            max_level=self.config.learning.curriculum.max_level,
            window_size=self.config.learning.curriculum.window_size,
            epsilon=self.config.learning.curriculum.epsilon,
            patience=self.config.learning.curriculum.patience
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
        postfix = "_norm" if self.config.env.normalize_reward or self.config.env.normalize_observation else ""
        name_log = f'{self.config.policy.name}_{self.config.learning.num_envs}_{self.config.env.name}{postfix}'
        
        checkpoint_callback = CheckpointCallback(
            save_freq=self.config.learning.save_model_frequency // self.config.learning.num_envs,  # fréquence = 1M steps globaux
            save_path="./model/",
            name_prefix=name_log+"_checkpoint"
        )

        curriculum_callback = CurriculumCallback(self.curriculum, self.env)

        self.model.learn(
            total_timesteps=self.config.learning.total_timesteps,
            log_interval=log_interval,
            tb_log_name=name_log,
            reset_num_timesteps=not self.use_model,
            callback=[checkpoint_callback, curriculum_callback] 
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
        postfix = "_norm" if node.config.env.normalize_reward or node.config.env.normalize_observation else ""
        name_log = f'{node.config.policy.name}_{node.config.learning.num_envs}_{node.config.env.name}{postfix}'
        log_dir = get_latest_log_dir(node.config.log.log_dir, name_log)

        os.makedirs(log_dir, exist_ok=True)
        with open(os.path.join(log_dir, "model_params.yaml"), "w") as f:
            yaml.dump(node.config.to_dict(), f)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    