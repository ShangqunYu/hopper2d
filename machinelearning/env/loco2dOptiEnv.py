import os, sys
sys.path.append('../build/machinelearning')
from loco2dOptiWrapper import Loco2dOptiWrapper
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Optional, Tuple, Union
import math

class Loco2dOptiEnv(gym.Env):
    def __init__(self):
        self.w = Loco2dOptiWrapper()
        act_lowbd =  np.array([-1, -1, -1, -1, -1, -1, -1],dtype=np.float32)
        act_highbd = np.array([ 1,  1,  1,  1,  1,  1,  1],dtype=np.float32)

        self.action_space = spaces.Box(low=act_lowbd, high=act_highbd, dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float64)


    def step(self, action):
        act = self.normalize_action(action)
        obs = self.w.step(act)
        # force the output for rdts and ldts to be close to 1
        reward = self.w.calc_reward()
        done = self.w.is_done()
        info = {}
        return obs, reward, done, False, info

    def reset(self, *,seed: Optional[int] = None, options: Optional[dict] = None,):
        super().reset(seed=seed)
        obs = self.w.reset()
        info = {}
        return obs, info

    def render(self, mode='human'):
        self.w.render()

    def seed(self, seed=None):
        pass

    def normalize_action(self, action):
        act = np.zeros(7)
        act[0] =  action[0] * 0.5 + 1     # desired_vel   range from 0.5 to 1.5
        act[1] =  action[1] + 1           # r_contact_loc range from 0 to 2    
        act[2] =  action[2] * 0.25 + 0.31   # r_contact_dts range from 0.02 to 0.52
        act[3] =  action[3] * 0.25 + 0.41 # r_flight_dts  range from 0.14 to 0.64
        act[4] =  action[4] * 0.4  + 0.5  # l_contact_loc range from 0.1 to 0.9
        act[5] =  action[5] * 0.15  + 0.21  # l_flight_dts  range from 0.01 to 0.41
        act[6] =  action[6] * 0.5  + 0.61  # l_contact_dts range from 0.11 to 0.91

        # print(action)
        return act


