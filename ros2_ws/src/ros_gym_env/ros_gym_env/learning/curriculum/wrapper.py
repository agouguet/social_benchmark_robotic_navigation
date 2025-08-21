from stable_baselines3.common.vec_env import VecEnvWrapper
import numpy as np
from collections import deque
import time

class RewardTrackingVecWrapper(VecEnvWrapper):
    def __init__(self, venv, curriculum, model=None, ros_node=None):
        super().__init__(venv)
        self.cumulative_rewards = np.zeros(venv.num_envs)
        self.completed_episode_rewards = []

        self.node = ros_node
        self.curriculum = curriculum
        self.model = model

    def reset(self):
        self.cumulative_rewards = np.zeros(self.num_envs)
        return self.venv.reset()

    def step_async(self, actions):
        self.venv.step_async(actions)

    def set_model(self, model):
        self.model = model

    def step_wait(self):
        obs, rewards, dones, infos = self.venv.step_wait()
        self.cumulative_rewards += rewards

        updated = False
        for i in range(self.num_envs):
            if dones[i]:
                total_reward = self.cumulative_rewards[i]
                steps_done = infos[i].get('episode_steps', 0)
                self.completed_episode_rewards.append(total_reward)
                self.cumulative_rewards[i] = 0.0

                # Plateau mode ou simplement reward pour update interne
                updated = self.curriculum.update(reward=total_reward, steps=steps_done, ros_node=self.node) or updated

        if updated:
            # Update des envs
            if hasattr(self.venv, "envs"):
                for env in self.venv.envs:
                    base_env = env.unwrapped
                    if hasattr(base_env, 'set_curriculum_level'):
                        base_env.set_curriculum_level(self.curriculum.level)

            # Update du learning rate du PPO
            if self.model is not None:
                new_lr = self.curriculum.get_lr()
                for param_group in self.model.policy.optimizer.param_groups:
                    param_group['lr'] = float(new_lr)
                if self.node:
                    self.node.get_logger().fatal(
                        f"[Curriculum] 🚀 Level {self.curriculum.level}, PPO LR set to {new_lr}"
                    )

        return obs, rewards, dones, infos