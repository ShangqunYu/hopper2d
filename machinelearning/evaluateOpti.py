import gymnasium as gym
from env.hopper2dOptiEnv import Hopper2dOptiEnv
import numpy as np
from stable_baselines3 import PPO, SAC
import math
gym.envs.register(
        id='Hopper2dOptiEnv-v0',
        entry_point='env:Hopper2dOptiEnv',
    )
env = gym.make('Hopper2dOptiEnv-v0')
obs,_ = env.reset()
model = SAC.load("./logs/optijumpJuly07sac/best_model.zip", print_system_info=True)
done = False
count = 0
totalReward = 0
while not done:
     
     act, _ = model.predict(obs, deterministic=True)
     action = np.array([0.0, 0.0, 0.0])
     action[0] =  act[0] + 1       # range from 0 to 2
     action[1] =  act[1] * 0.25 + 0.55
     action[2] =  act[2] * 0.25 + 0.55
     breakpoint()
     obs, reward, done, _, info = env.step(act)
     totalReward += reward
     env.render()

     count += 1
     # print(count, reward)
print(count)
print(totalReward)

