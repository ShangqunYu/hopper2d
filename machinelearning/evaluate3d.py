import gymnasium as gym
from env.hopper2dEnv import Hopper2dEnv
import numpy as np
from stable_baselines3 import PPO
import math
gym.envs.register(
     id='Hopper2dEnv-v0',
     entry_point='env:Hopper2dEnv',
)
env = gym.make('Hopper2dEnv-v0')
obs,_ = env.reset()
# breakpoint()
model = PPO.load("./logs/model/jumpJun21/model.zip", print_system_info=True)
done = False
count = 0
while not done:
     # breakpoint()
     act, _ = model.predict(obs, deterministic=True)
     # act = np.array([0.0, 0.0, 0.0])
     if count > 166:
          act = np.array([obs[3]-math.pi/6 , obs[4]+ math.pi/3, obs[5] - math.pi/6])
     obs, reward, done, _, info = env.step(act)
     # print(obs)
     env.render()
     # print(obs)
     # breakpoint()
     count += 1
     # print(count, reward)
print(count)

