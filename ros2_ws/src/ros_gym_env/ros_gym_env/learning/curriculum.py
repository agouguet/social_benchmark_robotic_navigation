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

    def update(self, new_reward):
        self.recent_rewards.append(new_reward)
        if len(self.recent_rewards) == self.recent_rewards.maxlen:
            avg = np.mean(self.recent_rewards)
            if self.prev_avg is not None:
                if abs(avg - self.prev_avg) < self.epsilon:
                    self.plateau_counter += 1
                else:
                    self.plateau_counter = 0
            self.prev_avg = avg

            if self.plateau_counter >= self.patience and self.level < self.max_level:
                self.level += 1
                self.plateau_counter = 0
                print(f"[Curriculum] Plateau détecté, passage au niveau {self.level}")
                return True  # indique qu'on doit changer de niveau

        return False  # pas encore prêt à changer



from stable_baselines3.common.callbacks import BaseCallback

class CurriculumCallback(BaseCallback):
    def __init__(self, curriculum_manager, envs, verbose=0):
        super().__init__(verbose)
        self.curriculum = curriculum_manager
        self.envs = envs

    def _on_step(self):
        # Prend la moyenne de la reward par épisode depuis les logs de Monitor
        if 'rollout/ep_rew_mean' in self.logger.name_to_value:
            rew = self.logger.name_to_value['rollout/ep_rew_mean']
            if self.curriculum.update(rew):
                # Notifie l'environnement
                if hasattr(self.envs, "envs"):
                    for env in self.envs.envs:
                        if hasattr(env, 'set_curriculum_level'):
                            env.set_curriculum_level(self.curriculum.level)
        return True
