from stable_baselines3.common.vec_env import VecEnvWrapper
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