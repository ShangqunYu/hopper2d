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
breakpoint()
model = PPO.load("./logs/model/jumpJun21/model.zip", print_system_info=True)
done = False
count = 0
totalReward = 0
while not done:
     # breakpoint()
     act, _ = model.predict(obs, deterministic=True)
     # act = np.array([0.0, 0.0, 0.0])
     obs, reward, done, _, info = env.step(act)
     theta1 = obs[3]
     theta2 = obs[4]
     theta3 = obs[5]
     totalReward += reward
     env.render()
     print("theta1:", theta1/math.pi,"pi" , "theta2:", theta2/math.pi, "pi", "theta3:",theta3/math.pi, "pi")

     count += 1
     # print(count, reward)
print(count)
print(totalReward)

