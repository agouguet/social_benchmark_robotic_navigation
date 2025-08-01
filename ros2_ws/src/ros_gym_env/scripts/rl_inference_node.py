#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from stable_baselines3 import PPO, SAC, DDPG
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import VecNormalize

from ros_gym_env.envs.env_factory import *
from ros_gym_env.envs.env_wrappers import *
from ros_gym_env.policy.policy_factory import *

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
        raise ValueError(f"RL Algo '{name}' non reconnue.")
    return algos[name]

class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')

        self.config_name = "training_002.yaml"  # <- adapte si besoin
        self.config = load_ros2_package_config("ros_gym_env", "config/" + self.config_name)

        self.model_path = os.path.join(get_package_share_directory("ros_gym_env"), "model/test2.zip")

        # --- Env construction ---
        env_fns = [self.make_env(0)]
        self.env = DummyVecEnv(env_fns)

        if self.config.env.normalize_observation or self.config.env.normalize_reward:
            self.env = VecNormalize.load(os.path.join(get_package_share_directory("ros_gym_env"), "model/vecnormalize.pkl"), self.env)
            self.env.training = False
            self.env.norm_reward = False

        self.env = VecEnvDelayWrapper(self.env, delay_sec=(0.05 / self.config.learning.speed_time))

        policy_class = method_factory[self.config.policy.name].get_policy(self.config)[0]
        rl_algo = get_rl_algo(self.config.algo.name)

        self.model = rl_algo.load(self.model_path, env=self.env)

        self.get_logger().info("Modèle chargé. Démarrage de l'inférence...")

    def make_env(self, env_id):
        def _init():
            env = env_factory[self.config.env.name](env_id, self.config, env_id_display_log=1)
            return env
        return _init

    def run_inference(self, episodes=20):
        for ep in range(episodes):
            obs = self.env.reset()
            done = False
            total_reward = 0
            steps = 0
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, reward, done, info = self.env.step(action)
                total_reward += reward
                steps += 1
            self.get_logger().info(f"[Episode {ep+1}] Reward: {total_reward} | Steps: {steps}")

        self.env.close()

def main(args=None):
    rclpy.init(args=args)
    node = RLInferenceNode()
    try:
        node.run_inference()
    except Exception as e:
        node.get_logger().error(f"Erreur durant l'inférence : {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
