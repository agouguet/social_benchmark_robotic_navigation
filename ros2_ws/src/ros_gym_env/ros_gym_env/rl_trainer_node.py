#!/usr/bin/env python3
import signal
import numpy as np
import os, time
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from ros_gym_env.ros_unity_gym_env import RosUnityEnv
from ros_gym_env.simple_env import RosUnitySimpleEnv
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from ros_gym_env.custom_cnn import *
import torch
import gymnasium as gym
from stable_baselines3.common.vec_env import VecEnvWrapper
from stable_baselines3.common.callbacks import EvalCallback

torch.cuda.empty_cache() 

class VecEnvDelayWrapper(VecEnvWrapper):
    def __init__(self, venv, delay_sec=0.1):
        super().__init__(venv)
        self.delay_sec = delay_sec

    def step_async(self, actions):
        return self.venv.step_async(actions)

    def step_wait(self):
        obs, rewards, dones, infos = self.venv.step_wait()
        time.sleep(self.delay_sec)  # <- Pause ici, après que tous les envs aient steppé
        return obs, rewards, dones, infos

    def reset(self):
        return self.venv.reset()

class ROSLoggingCallback(BaseCallback):
    def __init__(self, ros_node, print_freq: int = 1000, verbose=0):
        super().__init__(verbose)
        self.ros_node = ros_node
        self.print_freq = print_freq

    def _on_step(self) -> bool:
        if self.n_calls % self.print_freq == 0:
            # Accès au buffer d'épisodes récents
            ep_info = self.locals.get("infos", [{}])[-1].get("episode")
            if ep_info is not None:
                reward = ep_info["r"]
                length = ep_info["l"]
                self.ros_node.get_logger().info(
                    f"[Step {self.num_timesteps}] Episode reward: {reward:.2f} | Length: {length}"
                )
        return True

class RLTrainerNode(Node):
    def __init__(self):
        super().__init__('rl_trainer_node')
        max_iteration = 1024
        self.log_dir = "./log_drl/"

        num_envs = 6
        env_ids = [0, 1, 2, 3, 4, 5]  # paramètres spécifiques pour chaque env
        env_ids = [0, 1, 2, 3]

        env_fns = [self.make_env(i) for i in env_ids]
        # self.env = SubprocVecEnv(env_fns)  # ou DummyVectorEnv si debug/local
        self.env = DummyVecEnv(env_fns)
        self.env = VecEnvDelayWrapper(self.env, delay_sec=0.04)

        # self.env = RosUnitySimpleEnv(launch_ros_tcp=True, max_iteration=max_iteration)
        # self.env = RosUnityEnv(launch_ros_tcp=True, max_iteration=max_iteration)
        
        
        # policy parameters:
        policy_kwargs = dict(
            features_extractor_class=CustomCNN,
            features_extractor_kwargs=dict(features_dim=256),
            net_arch=[dict(pi=[256], vf=[128])]
        )

        self.model = PPO("CnnPolicy", self.env, policy_kwargs=policy_kwargs, learning_rate=1e-3, verbose=2, tensorboard_log=self.log_dir, n_steps=max_iteration, n_epochs=10, batch_size=32)
        # self.model = PPO("MlpPolicy", self.env, verbose=1, n_steps=100000, tensorboard_log=self.log_dir)
        
    def make_env(self, env_id):
        def _init():
            # env = RosUnitySimpleEnv(env_id)
            env = RosUnityEnv(env_id)
            env = Monitor(env, self.log_dir+str(env_id)+"/")
            return env
        return _init

    def start_training(self):
        self.get_logger().info("Start training ...")
        total_timesteps = 2000000
        print_freq = 10

        eval_callback = EvalCallback(self.env,
                             best_model_save_path=self.log_dir,
                             log_path=self.log_dir,
                             eval_freq=10_000,
                             deterministic=True,
                             render=False)

        callbacks = [
            ROSLoggingCallback(self, print_freq=print_freq),
            eval_callback
        ]
        self.model.learn(total_timesteps=2000000, log_interval=20, tb_log_name='drl_vo_policy', callback=CallbackList(callbacks), reset_num_timesteps=True)
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
    