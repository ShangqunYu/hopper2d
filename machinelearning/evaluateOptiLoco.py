import gymnasium as gym
from env.loco2dOptiEnv import Loco2dOptiEnv
import numpy as np
from stable_baselines3 import PPO, SAC
import math
gym.envs.register(
        id='Loco2dOptiEnv-v0',
        entry_point='env:Loco2dOptiEnv',
    )
env = gym.make('Loco2dOptiEnv-v0')
obs,_ = env.reset()
model = SAC.load("./logs/optijumpJuly14/best_model.zip", print_system_info=True)
done = False
count = 0
totalReward = 0
while not done:
    
    act, _ = model.predict(obs, deterministic=True)
    action = np.zeros(6)
    action[0] =  act[0] + 1       # range from 0 to 2
    action[1] =  act[1] * 0.2 + 0.3 # range from 0.1 to 0.5
    action[2] =  act[2] * 0.25 + 0.55 # range from 0.3 to 0.8
    action[3] =  act[3] * 0.4  + 0.4  # range from 0.0 to 0.8
    action[4] =  act[4] * 0.2  + 0.3  # range from 0.1 to 0.8
    action[5] =  act[5] * 0.3  + 0.5  # range from 0.2 to 0.8
    breakpoint()
    obs, reward, done, _, info = env.step(act)
    totalReward += reward
    env.render()

    count += 1
    # print(count, reward)
print(count)
print(totalReward)

