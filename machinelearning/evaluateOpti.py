import gymnasium as gym
from env.hopper2dOptiEnv import Hopper2dOptiEnv
import numpy as np
from stable_baselines3 import PPO
import math
gym.envs.register(
        id='Hopper2dOptiEnv-v0',
        entry_point='env:Hopper2dOptiEnv',
    )
env = gym.make('Hopper2dOptiEnv-v0')
obs,_ = env.reset()
model = PPO.load("./logs/optijumpJuly05/best_model.zip", print_system_info=True)
done = False
count = 0
totalReward = 0
while not done:
     # breakpoint()
     act, _ = model.predict(obs, deterministic=True)
     breakpoint()
     obs, reward, done, _, info = env.step(act)
     totalReward += reward
     env.render()

     count += 1
     # print(count, reward)
print(count)
print(totalReward)

