import gymnasium as gym
from env.loco3dEnv import Loco3dEnv
import numpy as np
from stable_baselines3 import PPO

gym.envs.register(
     id='Loco3dEnv-v0',
     entry_point='env:Loco3dEnv',
)
env = gym.make('Loco3dEnv-v0')
obs,_ = env.reset()

model = PPO.load("./logs/model/loco3d_cont_june6/loco3d__800000_steps", print_system_info=True)
done = False
while not done:
     breakpoint()
     act, _ = model.predict(obs, deterministic=True)
     obs, reward, done, _, info = env.step(act)
     env.render()


