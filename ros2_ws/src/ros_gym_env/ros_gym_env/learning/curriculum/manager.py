import time
import numpy as np
from collections import deque

class CurriculumManager:
    def __init__(self,
                 config):
        """
        mode : "steps", "time" or "plateau"
        - steps   : progression based on the total number of steps (requires level_thresholds)
        - time    : progression based on elapsed time (requires level_thresholds in seconds)
        - plateau : progression based on reward stagnation

        level_thresholds : list of tuples (min, max, level) 
                           expressed in steps if mode="steps"
                           expressed in seconds if mode="time"
        """
        # self.config = config
        if hasattr(config, "to_dict"):
            config = config.to_dict()

        self.mode = config.get("mode", "plateau")
        self.max_level = config.get("max_level", 3)

        # Steps & time thresholds
        self.level_thresholds = config.get("thresholds", [])
        self.level = 0
        self.total_steps = 0
        self.start_time = time.time()

        # Plateau params
        self.recent_rewards = deque(maxlen=config.get("window_size", 10))
        self.prev_avg = None
        self.plateau_counter = 0
        self.epsilon = config.get("epsilon", 1.0)
        self.patience = config.get("patience", 3)

        # Learning rate par niveau
        self.learning_rate_per_level = config.get("learning_rate_per_level", {})
        self.learning_rate_per_level = {int(k): v for k, v in self.learning_rate_per_level.items()}
        self.default_lr = config.get("default_learning_rate", 5e-5)

    def get_lr(self):
        """Retourne le learning rate correspondant au niveau actuel"""
        return self.learning_rate_per_level.get(self.level, self.default_lr)

    def update(self, reward=None, steps=None, ros_node=None):
        updated = False

        if self.mode == "steps":
            self.total_steps += steps
            for (min_s, max_s, lvl) in self.level_thresholds:
                if min_s <= self.total_steps < max_s and lvl != self.level:
                    self.level = lvl
                    updated = True
                    break

        elif self.mode == "time":
            elapsed = time.time() - self.start_time
            for (min_t, max_t, lvl) in self.level_thresholds:
                if min_t <= elapsed < max_t and lvl != self.level:
                    self.level = lvl
                    updated = True
                    break

        elif self.mode == "plateau":
            if reward is not None:
                self.recent_rewards.append(reward)
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
                        updated = True
                        
        return updated
