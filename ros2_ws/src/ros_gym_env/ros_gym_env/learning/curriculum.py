from collections import deque
import numpy as np

class CurriculumManager:
    def __init__(self, max_level=3, window_size=10, epsilon=1.0, patience=3):
        self.level = 0
        self.max_level = max_level
        self.recent_rewards = deque(maxlen=window_size)
        self.prev_avg = None
        self.plateau_counter = 0
        self.epsilon = epsilon
        self.patience = patience

    def update(self, new_reward, ros_node=None):
        self.recent_rewards.append(new_reward)
        if len(self.recent_rewards) == self.recent_rewards.maxlen:
            avg = np.mean(self.recent_rewards)
            if self.prev_avg is not None:
                ros_node.get_logger().fatal("ABS/EPS ==> {} {}".format(abs(avg - self.prev_avg), self.epsilon))
                if abs(avg - self.prev_avg) < self.epsilon:
                    self.plateau_counter += 1
                else:
                    self.plateau_counter = 0
            self.prev_avg = avg

            if self.plateau_counter >= self.patience and self.level < self.max_level:
                self.level += 1
                self.plateau_counter = 0
                print(f"[Curriculum] Plateau détecté, passage au niveau {self.level}")
                return True 

        return False



from stable_baselines3.common.callbacks import BaseCallback

class CurriculumCallback(BaseCallback):
    def __init__(self, curriculum_manager, envs, verbose=0, ros_node=None):
        super().__init__(verbose)
        self.curriculum = curriculum_manager
        self.envs = envs
        self.ros_node = ros_node

    def _on_rollout_end(self):
        self.logger.record('curriculum/avg', float(self.curriculum.prev_avg or 0.0))
        self.logger.record('curriculum/epsilon', float(self.curriculum.epsilon))
        self.logger.record('curriculum/level', self.curriculum.level)

    def _on_step(self):
        # Nécessaire même si non utilisé
        return True
    


class PolicyUpdateCallback(BaseCallback):
    def __init__(self, verbose=0, ros_node=None):
        super().__init__(verbose)
        self.ros_node = ros_node

    def _on_rollout_start(self) -> None:
        # Appelé juste avant l’update
        if hasattr(self.training_env, 'envs'):
            for env in self.training_env.envs:
                base_env = env.unwrapped
                if hasattr(base_env, 'on_policy_update_start'):
                    base_env.on_policy_update_start()

    def _on_rollout_end(self) -> None:
        # Appelé juste après l’update
        if hasattr(self.training_env, 'envs'):
            for env in self.training_env.envs:
                base_env = env.unwrapped
                if hasattr(base_env, 'on_policy_update_end'):
                    base_env.on_policy_update_end()
    
    def _on_step(self):
        # Nécessaire même si non utilisé
        return True
    
class SuccessRateCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.successes = []
    
    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        for info in infos:
            if "is_success" in info:
                self.successes.append(1 if info["is_success"] else 0)
        return True

    def _on_rollout_end(self) -> None:
        if len(self.successes) > 0:
            success_rate = np.mean(self.successes)
            self.logger.record("eval/success_rate", success_rate)
            self.successes.clear()