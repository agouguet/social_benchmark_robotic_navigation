#!/usr/bin/env python3
import signal
import numpy as np
import os
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from ros_gym_env.ros_unity_gym_env import RosUnityEnv
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from ros_gym_env.custom_cnn import *


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, node, check_freq: int, log_dir: str, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf
        self.node = node

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
          os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

          # Retrieve training reward
          x, y = ts2xy(load_results(self.log_dir), 'timesteps')
          if len(x) > 0:
              # Mean training reward over the last 100 episodes
              mean_reward = np.mean(y[-100:])
              if self.verbose > 0:
                self.node.get_logger().info("Num timesteps: {}".format(self.num_timesteps))
                self.node.get_logger().info("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward, mean_reward))

              # New best model, you could save the agent here
              if mean_reward > self.best_mean_reward:
                  self.best_mean_reward = mean_reward
                  # Example for saving best model
                  if self.verbose > 0:
                    self.node.get_logger().info("Saving new best model to {}".format(self.save_path))
                  self.model.save(self.save_path)
                  
        # save model every 100000 timesteps:
        if self.n_calls % (50000) == 0:
          # Retrieve training reward
          path = self.save_path + '_model' + str(self.n_calls)
          self.model.save(path)
	  
        return True

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
        self.env = RosUnityEnv(launch_ros_tcp=True, max_iteration=max_iteration)
        self.log_dir = "."
        

        # policy parameters:
        policy_kwargs = dict(
            features_extractor_class=CustomCNN,
            features_extractor_kwargs=dict(features_dim=256),
            net_arch=[dict(pi=[256], vf=[128])]
        )

        self.model = PPO("CnnPolicy", self.env, policy_kwargs=policy_kwargs, learning_rate=1e-3, verbose=2, tensorboard_log=self.log_dir, n_steps=max_iteration, n_epochs=10, batch_size=128)
        # self.model = PPO("MlpPolicy", self.env, verbose=1, n_steps=100000, tensorboard_log="./ppo_robot_tensorboard/")
        
        # self.env = Monitor(self.env, self.log_dir)

    def start_training(self):
        self.get_logger().info("Start training ...")
        total_timesteps = 2000000
        print_freq = 10

        callbacks = [
            SaveOnBestTrainingRewardCallback(self, check_freq=2500, log_dir=self.log_dir),
            ROSLoggingCallback(self, print_freq=print_freq),
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
    