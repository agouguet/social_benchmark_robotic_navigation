#!/usr/bin/env python3
import time
import numpy as np
import multiprocessing as mp
from stable_baselines3 import PPO
from ros_gym_env.ros_unity_gym_env import RosUnityEnv
from ros_gym_env.ros_gym_env.custom_cnn_full import CustomCNN
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
import torch
import gymnasium as gym
from stable_baselines3.common.vec_env import VecEnvWrapper
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.vec_env import VecEnv

torch.cuda.empty_cache() 

class ThreadedVecEnv(VecEnv):
    def __init__(self, env_fns):
        self.envs = [fn() for fn in env_fns]
        self.n_envs = len(self.envs)
        self.observation_space = self.envs[0].observation_space
        self.action_space = self.envs[0].action_space

        self._obs = [None] * self.n_envs
        self._rewards = [None] * self.n_envs
        self._dones = [None] * self.n_envs
        self._infos = [None] * self.n_envs

        super().__init__(self.n_envs, self.observation_space, self.action_space)

    def reset(self):
        return [env.reset() for env in self.envs]

    def step_async(self, actions):
        self._actions = actions

    def step_wait(self):
        threads = []
        def worker(i, action):
            obs, rew, done, info = self.envs[i].step(action)
            if done:
                obs = self.envs[i].reset()
            self._obs[i], self._rewards[i], self._dones[i], self._infos[i] = obs, rew, done, info

        for i, action in enumerate(self._actions):
            t = threading.Thread(target=worker, args=(i, action))
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        return self._obs, self._rewards, self._dones, self._infos
    
    def close(self):
        for env in self.envs:
            env.close()

    def env_is_wrapped(self, wrapper_class, indices=None):
        if indices is None:
            indices = range(self.n_envs)
        return [isinstance(self.envs[i], wrapper_class) for i in indices]

    def env_method(self, method_name, *args, indices=None, **kwargs):
        if indices is None:
            indices = range(self.n_envs)
        results = []
        for i in indices:
            method = getattr(self.envs[i], method_name)
            results.append(method(*args, **kwargs))
        return results

    def get_attr(self, attr_name, indices=None):
        if indices is None:
            indices = range(self.n_envs)
        return [getattr(self.envs[i], attr_name) for i in indices]

    def set_attr(self, attr_name, value, indices=None):
        if indices is None:
            indices = range(self.n_envs)
        for i in indices:
            setattr(self.envs[i], attr_name, value)








from stable_baselines3.common.vec_env import VecEnv
import numpy as np

class AsyncEnvManager(VecEnv):
    def __init__(self, env_fns):
        self.n_envs = len(env_fns)
        self.remotes, self.work_remotes = zip(*[mp.Pipe() for _ in range(self.n_envs)])
        self.ps = []
        for work_remote, remote, env_fn in zip(self.work_remotes, self.remotes, env_fns):
            p = mp.Process(target=env_worker, args=(work_remote, remote, env_fn))
            p.daemon = True
            p.start()
            work_remote.close()
            self.ps.append(p)
        
        # On récupère la première observation pour avoir l'espace d'observation
        self._obs = self.reset()

        # Tu dois récupérer les espaces observation/action de ton premier env
        # Par exemple ici on appelle une méthode pour ça sur le premier env (à adapter)
        # Ici juste un placeholder, adapte selon ton env
        dummy_env = env_fns[0]()
        self.observation_space = dummy_env.observation_space
        self.action_space = dummy_env.action_space
        dummy_env.close()

    def step_async(self, actions):
        for remote, action in zip(self.remotes, actions):
            remote.send(('step', action))

    def step_wait(self):
        results = [remote.recv() for remote in self.remotes]
        obs, rewards, dones, infos = zip(*results)
        # Convertir en np.array si nécessaire
        return np.stack(obs), np.array(rewards), np.array(dones), infos

    def reset(self):
        for remote in self.remotes:
            remote.send(('reset', None))
        obs = [remote.recv() for remote in self.remotes]
        self._obs = obs
        return np.stack(obs)

    def close(self):
        for remote in self.remotes:
            remote.send(('close', None))
        for p in self.ps:
            p.join()

    def render(self):
        pass  # Optionnel, implémente si besoin

    # Ces méthodes sont nécessaires pour VecEnv
    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False for _ in range(self.n_envs)]

    def env_method(self, method_name, *args, indices=None, **kwargs):
        # Exécuter une méthode sur tous les environnements sélectionnés
        if indices is None:
            indices = range(self.n_envs)
        results = []
        for i in indices:
            self.remotes[i].send(('call_method', (method_name, args, kwargs)))
        for i in indices:
            results.append(self.remotes[i].recv())
        return results

    def get_attr(self, attr_name, indices=None):
        if indices is None:
            indices = range(self.n_envs)
        results = []
        for i in indices:
            self.remotes[i].send(('get_attr', attr_name))
        for i in indices:
            results.append(self.remotes[i].recv())
        return results

    def set_attr(self, attr_name, value, indices=None):
        if indices is None:
            indices = range(self.n_envs)
        for i in indices:
            self.remotes[i].send(('set_attr', (attr_name, value)))
        for i in indices:
            _ = self.remotes[i].recv()


def env_worker(remote, parent_remote, env_fn_wrapper):
    parent_remote.close()
    env = env_fn_wrapper()
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == 'step':
                obs, reward, done, info = env.step(data)
                if done:
                    obs = env.reset()
                remote.send((obs, reward, done, info))
            elif cmd == 'reset':
                obs = env.reset()
                remote.send(obs)
            elif cmd == 'close':
                remote.close()
                break
            elif cmd == 'call_method':
                method_name, args, kwargs = data
                result = getattr(env, method_name)(*args, **kwargs)
                remote.send(result)
            elif cmd == 'get_attr':
                result = getattr(env, data)
                remote.send(result)
            elif cmd == 'set_attr':
                attr_name, value = data
                setattr(env, attr_name, value)
                remote.send(None)
            else:
                raise NotImplementedError
    except KeyboardInterrupt:
        pass











def make_env(env_id, logdir, max_iteration):
    def _init():
        env = RosUnityEnv(env_id, max_iteration=max_iteration)
        env = Monitor(env, logdir+str(env_id)+"/")
        return env
    return _init


class MyCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        # ici tu mets ton code à exécuter à chaque étape
        print("Step:", self.num_timesteps)
        return True

def launch_training():

    last_time = time.time()

    def on_step_callback(_locals, _globals):
        nonlocal last_time
        now = time.time()
        delta = now - last_time
        last_time = now
        print(f"Step duration since last call: {delta:.3f}s")
        return True

    max_iteration = 512
    log_dir = "./log_drl/"

    num_envs = 4
    env_ids = np.arange(num_envs)

    env_fns = [make_env(i, log_dir, max_iteration) for i in env_ids]
    env = SubprocVecEnv(env_fns)
    # env = AsyncEnvManager([make_env(i, log_dir, max_iteration) for i in range(num_envs)])
    # env_fns = [make_env(i, log_dir, max_iteration) for i in range(num_envs)]
    # env = ThreadedVecEnv(env_fns)

    # policy parameters:
    # policy_kwargs = dict(
    #     features_extractor_class=CustomCNN,
    #     features_extractor_kwargs=dict(features_dim=256),
    #     net_arch=[dict(pi=[256], vf=[128])]
    # )

    policy_kwargs = dict(
        features_extractor_class=CustomCNN,
        features_extractor_kwargs=dict(features_dim=64),  # réduit à 64
        net_arch=[dict(pi=[64], vf=[64])]                 # architectures plus petites
    )

    # model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs, learning_rate=1e-3, verbose=2, tensorboard_log=log_dir, n_steps=max_iteration, n_epochs=10, batch_size=256)
    model = PPO("CnnPolicy", env, policy_kwargs=policy_kwargs,
            learning_rate=1e-3, verbose=2, tensorboard_log=log_dir,
            n_steps=128, n_epochs=5, batch_size=64)  # batch et steps réduits
            
    eval_callback = EvalCallback(env,
                            best_model_save_path=log_dir,
                            log_path=log_dir,
                            eval_freq=int(10_000/num_envs),
                            deterministic=True,
                            render=False)

    callbacks = [
        eval_callback,
        # MyCallback()
    ]
    model.learn(total_timesteps=2000000, log_interval=20, tb_log_name='drl_vo_policy', callback=CallbackList(callbacks), reset_num_timesteps=True)
    model.save("drl_vo_model")
    env.close()