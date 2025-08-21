import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

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