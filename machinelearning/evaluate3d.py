import gymnasium as gym
from env.hopper2dEnv import Hopper2dEnv
import numpy as np
from stable_baselines3 import PPO

gym.envs.register(
     id='Hopper2dEnv-v0',
     entry_point='env:Hopper2dEnv',
)
env = gym.make('Hopper2dEnv-v0')
obs,_ = env.reset()

model = PPO.load("./logs/model/hopper2dJun17/NoAuxPenalty_1600000_steps.zip", print_system_info=True)
done = False
count = 0
while not done:
     act, _ = model.predict(obs, deterministic=True)
     # act = np.array([0.0, 0.0, 0.0])
     obs, reward, done, _, info = env.step(act)
     env.render()
     count += 1
     print(count)

