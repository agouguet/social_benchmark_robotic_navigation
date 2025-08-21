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

