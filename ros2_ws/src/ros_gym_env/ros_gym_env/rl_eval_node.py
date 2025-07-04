#!/usr/bin/env python3
import rclpy
from ros_gym_env.ros_unity_gym_env import RosUnityEnv
from stable_baselines3 import PPO
import time

def main():
    rclpy.init()
    env = RosUnityEnv(launch_ros_tcp=True)
    model = PPO.load('best_model_model1120000', env=env)

    obs, info = env.reset()
    done = False

    print("Début de l'inférence")

    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)

        # Optionnel : limiter la fréquence d'action
        time.sleep(0.05)

    print("Fin de l'épisode")
    env.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
