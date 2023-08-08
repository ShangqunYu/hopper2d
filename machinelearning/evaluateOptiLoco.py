import gymnasium as gym
from env.loco2dOptiEnv import Loco2dOptiEnv
import numpy as np
from stable_baselines3 import PPO, SAC
import math
import torch

def visualize_contact(action):
    dt = 0.06
    r_contact_hor = math.floor(action[2] / dt) 
    r_flight_hor =  math.floor(action[3] / dt)
    l_flight_hor =  math.floor(action[5] / dt)
    l_contact_hor = math.floor(action[6] / dt)
    pred_hor = r_contact_hor * 2 + r_flight_hor
    output = np.zeros([2,pred_hor])
    output[0,0:r_contact_hor] = 1
    output[0,r_contact_hor+r_flight_hor : pred_hor] = 1
    output[1,l_flight_hor:l_flight_hor+ l_contact_hor] = 1
    print(output)


gym.envs.register(
        id='Loco2dOptiEnv-v0',
        entry_point='env:Loco2dOptiEnv',
    )
env = gym.make('Loco2dOptiEnv-v0')
obs,_ = env.reset()
# breakpoint()
# model = SAC.load("./logs/optijumpJuly18/opti_960000_steps.zip")
model = SAC.load("./logs/optijumpAug6/best_model.zip", env=env, custom_objects = {'observation_space': env.observation_space, 'action_space': env.action_space})

done = False
count = 0
totalReward = 0
while not done:
    
    act, _ = model.predict(obs, deterministic=True)
    breakpoint()
    # act = np.zeros(7)
    action = np.zeros(7)
    action[0] =  act[0] * 0.5 + 1     # desired_vel   range from 0.5 to 1.5
    action[1] =  act[1] + 1           # r_contact_loc range from 0 to 2    
    action[2] =  act[2] * 0.3 + 0.27   # r_contact_dts range from 0.03 to 0.57
    action[3] =  act[3] * 0.25 + 0.39 # r_flight_dts  range from 0.14 to 0.64
    action[4] =  act[4] * 0.4  + 0.5  # l_contact_loc range from 0.0 to 0.8
    action[5] =  act[5] * 0.2  + 0.21  # l_flight_dts  range from 0.01 to 0.41
    action[6] =  act[6] * 0.4  + 0.51   # l_contact_dts range from  0.11 to 0.91
    visualize_contact(action)
    obs, reward, done, _, info = env.step(act)
    totalReward += reward
    env.render()

    count += 1
    # print(count, reward)
print(count)
print(totalReward)

