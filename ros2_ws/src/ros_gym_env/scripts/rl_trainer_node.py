#!/usr/bin/env python3
import signal
import numpy as np
import os, time
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from ros_gym_env.ros_unity_gym_env import RosUnityEnv
from ros_gym_env.ros_unity_gym_env_test import RosUnityEnvTest
from ros_gym_env.simple_env import RosUnitySimpleEnv
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from ros_gym_env.custom_cnn_full import *
from ros_gym_env.my_custom_cnn_full import *
import torch
import gymnasium as gym
from stable_baselines3.common.vec_env import VecEnvWrapper
from stable_baselines3.common.callbacks import EvalCallback
from collections import deque

torch.cuda.empty_cache() 

class VecEnvDelayWrapper(VecEnvWrapper):
    def __init__(self, venv, delay_sec=0.1, ros_node=None):
        super().__init__(venv)
        self.delay_sec = delay_sec
        self.last_step_time = None  # Pour stocker le temps du dernier step
        self.node = ros_node
        self.step_durations = deque(maxlen=100)

    def step_async(self, actions):
        return self.venv.step_async(actions)

    def step_wait(self):
        # Mesure de temps : début
        now = time.perf_counter()

        # Calcul du delta de temps
        if self.last_step_time is not None:
            delta = now - self.last_step_time
            self.step_durations.append(delta)
            avg = sum(self.step_durations) / len(self.step_durations)

            # if (self.node is not None):
            #     self.node.get_logger().fatal(f"[VecEnvDelayWrapper] Step Δt: {delta:.4f}s | Avg (last 100): {avg:.4f}s")
            # else:
            #     print(f"[VecEnvDelayWrapper] Step Δt: {delta:.4f}s | Avg (last 100): {avg:.4f}s")
        self.last_step_time = now

        obs, rewards, dones, infos = self.venv.step_wait()
        # time.sleep(self.delay_sec)  # <- Pause ici, après que tous les envs aient steppé
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
    
class MyCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.reward_buffer = []  # Stocke les récompenses depuis le dernier print
        self.save_path = "./log_drl/"

    def _on_step(self) -> bool:
        steps = self.num_timesteps
        actions = self.locals.get("actions", None)
        obs = self.locals.get("new_obs", None)
        rewards = self.locals.get("rewards", None)
        dones = self.locals.get("dones", None)

        # Accumuler les récompenses
        if rewards is not None:
            self.reward_buffer.extend(rewards)

        if steps % 10000 == 0 and self.reward_buffer:
            sum_reward = np.sum(self.reward_buffer)
            avg_reward = np.mean(self.reward_buffer)
            print(f"Step {steps} | Reward (sum/avg): {sum_reward:.3f}   {avg_reward:.3f}")
            self.logger.record("custom/episode_reward", avg_reward)
            self.logger.record("custom/episode_reward_sum", sum_reward)
            self.reward_buffer.clear()  # Réinitialise le buffer

        if steps % 200000 == 0:
            model_filename = f"{self.save_path}_{steps}_steps"
            self.model.save(model_filename)
            print(f"✅ Modèle sauvegardé : {model_filename}")

        # Debug NaN
        if obs is not None and np.any(np.isnan(obs)):
            print("⚠️ NaN detected in observation:", obs)
        if actions is not None and np.any(np.isnan(actions)):
            print("⚠️ NaN detected in action:", actions)
        if rewards is not None and np.any(np.isnan(rewards)):
            print("⚠️ NaN detected in reward:", rewards)

        return True

class RLTrainerNode(Node):
    def __init__(self):
        super().__init__('rl_trainer_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_iteration', 512),
                ('log_dir', "./log_drl/"),
                ('log_interval', 1),
                ('num_envs', 1),
                ('speed_time', 1.0)
            ]
        )

        self.max_iteration = self.get_parameter('max_iteration').value
        self.log_dir = self.get_parameter('log_dir').value
        self.log_interval = self.get_parameter('log_interval').value
        self.num_envs = self.get_parameter('num_envs').value
        self.speed_time = self.get_parameter('speed_time').value

        env_ids = np.arange(self.num_envs)
        base_delay_time = 0.05

        env_fns = [self.make_env(i) for i in env_ids]
        self.env = DummyVecEnv(env_fns)
        self.env = VecEnvDelayWrapper(self.env, delay_sec=(base_delay_time/self.speed_time))

        # policy parameters:
        policy_kwargs = dict(
            features_extractor_class=Scan1DCNN,
            features_extractor_kwargs=dict(features_dim=256),
            # net_arch=[dict(pi=[256], vf=[128])]
        )

        # policy_kwargs = dict(
        #     features_extractor_class=MyCustomCNN,
        #     features_extractor_kwargs=dict(features_dim=256),
        #     net_arch=[dict(pi=[256], vf=[128])]
        # )

        # policy_kwargs = dict(
        #     features_extractor_class=CustomCNN,
        #     features_extractor_kwargs=dict(features_dim=256),
        #     net_arch=[dict(pi=[256], vf=[128])]
        # )

        self.model = PPO("CnnPolicy", self.env, policy_kwargs=policy_kwargs, learning_rate=3e-4, verbose=2, tensorboard_log=self.log_dir, n_steps=self.max_iteration, n_epochs=10, batch_size=256)
        # self.model = PPO("MlpPolicy", self.env, learning_rate=1e-3, verbose=2, tensorboard_log=self.log_dir, n_steps=self.max_iteration, n_epochs=10, batch_size=256)

    def make_env(self, env_id):
        def _init():
            # env = RosUnitySimpleEnv(env_id)
            # env = RosUnityEnv(env_id, max_iteration=self.max_iteration)
            env = RosUnityEnvTest(env_id, max_iteration=self.max_iteration, env_id_display_log=0)
            env = Monitor(env, self.log_dir+str(env_id)+"/")
            return env
        return _init

    def start_training(self):
        self.get_logger().info("Start training ...")
        total_timesteps = 1e8
        print_freq = 10

        eval_callback = EvalCallback(self.env,
                             best_model_save_path=self.log_dir,
                             log_path=self.log_dir,
                             eval_freq=int(10_000/self.num_envs),
                             deterministic=True,
                             render=False)
        callbacks = [
            ROSLoggingCallback(self, print_freq=print_freq),
            MyCallback()
            # eval_callback
        ]
        log_interval = max(1, int(self.log_interval / self.num_envs))
        self.model.learn(total_timesteps=total_timesteps, log_interval=log_interval, tb_log_name='drl_vo_policy_{}'.format(self.num_envs), callback=CallbackList(callbacks), reset_num_timesteps=True)
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
    


# #!/usr/bin/env python3
# import signal
# import numpy as np
# import os, time
# from tqdm import tqdm
# import rclpy
# from rclpy.node import Node
# from stable_baselines3 import PPO
# from ros_gym_env.ros_unity_gym_env import RosUnityEnv
# from ros_gym_env.simple_env import RosUnitySimpleEnv
# from ros_gym_env.train_rl import launch_training
# from stable_baselines3.common.env_checker import check_env
# from stable_baselines3.common.monitor import Monitor
# from stable_baselines3 import PPO
# from stable_baselines3.common.results_plotter import load_results, ts2xy
# from stable_baselines3.common.vec_env import DummyVecEnv
# from stable_baselines3.common.vec_env import SubprocVecEnv
# from stable_baselines3.common.callbacks import BaseCallback, CallbackList
# from ros_gym_env.custom_cnn import *
# import torch
# import gymnasium as gym
# from stable_baselines3.common.vec_env import VecEnvWrapper
# from stable_baselines3.common.callbacks import EvalCallback

# torch.cuda.empty_cache() 

# class RLTrainerNode(Node):
#     def __init__(self):
#         super().__init__('rl_trainer_node')
#         launch_training()

# def main(args=None):
#     rclpy.init(args=args)
#     node = RLTrainerNode()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
    