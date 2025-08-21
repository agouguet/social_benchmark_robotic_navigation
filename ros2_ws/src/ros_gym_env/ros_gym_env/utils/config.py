import os, yaml, glob
from stable_baselines3 import PPO, SAC, DDPG
from ament_index_python.packages import get_package_share_directory

class Config:
    def __init__(self, d):
        for k, v in d.items():
            if isinstance(v, dict):
                v = Config(v)
            setattr(self, k, v)

    def to_dict(self):
        result = {}
        for k, v in self.__dict__.items():
            if isinstance(v, Config):
                result[k] = v.to_dict()
            else:
                result[k] = v
        return result


def get_latest_log_dir(base_dir, name_log):
        """
        Retourne le dernier dossier créé correspondant à base_dir/name_log*
        """
        pattern = os.path.join(base_dir, f"{name_log}*")
        dirs = [d for d in glob.glob(pattern) if os.path.isdir(d)]
        if not dirs:
            return os.path.join(base_dir, name_log)  # dossier de base
        latest_dir = max(dirs, key=os.path.getmtime)
        return latest_dir


def load_ros2_package_config(package_name, relative_path):
    pkg_path = get_package_share_directory(package_name)
    config_path = os.path.join(pkg_path, relative_path)
    with open(config_path, "r") as f:
        data = yaml.safe_load(f)
    return Config(data)


def get_rl_algo(name):
    name = name.lower()
    algos = {
        "ppo": PPO,
        "sac": SAC,
        "ddpg": DDPG
    }
    if name not in algos:
        raise ValueError(f"RL Algo '{name}' non reconnue. Choisis parmi : {list(algos.keys())}")
    return algos[name]