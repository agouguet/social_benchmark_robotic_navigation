from stable_baselines3.common.vec_env import VecEnvWrapper
import numpy as np
from collections import deque
import time

class VecEnvDelayWrapper(VecEnvWrapper):
    def __init__(self, venv, delay_sec=0.1, ros_node=None):
        super().__init__(venv)
        self.delay_sec = delay_sec
        # self.last_step_time = None  # Pour stocker le temps du dernier step
        # self.node = ros_node
        # self.step_durations = deque(maxlen=100)

    def step_async(self, actions):
        return self.venv.step_async(actions)

    def step_wait(self):
        # Mesure de temps : début
        # now = time.perf_counter()

        # # Calcul du delta de temps
        # if self.last_step_time is not None:
        #     delta = now - self.last_step_time
        #     self.step_durations.append(delta)
            
        # self.last_step_time = now

        obs, rewards, dones, infos = self.venv.step_wait()
        # time.sleep(self.delay_sec)  # <- Pause ici, après que tous les envs aient steppé
        return obs, rewards, dones, infos

    def reset(self):
        return self.venv.reset()
    


class RewardTrackingVecWrapper(VecEnvWrapper):
    def __init__(self, venv, curriculum, ros_node=None):
        super().__init__(venv)
        self.cumulative_rewards = np.zeros(venv.num_envs)
        self.completed_episode_rewards = []

        self.node = ros_node
        self.curriculum = curriculum

    def reset(self):
        self.cumulative_rewards = np.zeros(self.num_envs)
        return self.venv.reset()

    def step_async(self, actions):
        self.venv.step_async(actions)

    def step_wait(self):
        obs, rewards, dones, infos = self.venv.step_wait()
        self.cumulative_rewards += rewards

        updated = False

        for i in range(self.num_envs):
            if dones[i]:
                total_reward = self.cumulative_rewards[i]
                self.completed_episode_rewards.append(total_reward)
                self.cumulative_rewards[i] = 0.0

                # if self.node:
                #     self.node.get_logger().fatal(f"[Curriculum] Env {i} episode reward: {total_reward}")

                # ⚠️ On ne met à jour le curriculum que si un épisode est terminé
                updated = self.curriculum.update(total_reward, self.node) or updated

        # 🎯 Si le curriculum a progressé, on met à jour tous les envs
        if updated:
            if hasattr(self.venv, "envs"):
                for env in self.venv.envs:
                    if hasattr(env, 'set_curriculum_level'):
                        env.set_curriculum_level(self.curriculum.level)
                if self.node:
                    self.node.get_logger().fatal(f"[Curriculum] 🚀 Passage global au niveau {self.curriculum.level}")

        return obs, rewards, dones, infos