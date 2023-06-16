import os, sys
sys.path.append('../build/machinelearning')
from hopper2dWrapper import Hopper2dWrapper
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Optional, Tuple, Union
import math

class Hopper2dEnv(gym.Env):
    def __init__(self):
        self.w = Hopper2dWrapper()
        act_lowbd = np.ones(3,dtype=np.float32)  * -3
        act_highbd = np.ones(3,dtype=np.float32) *  3
        self.action_space = spaces.Box(low=act_lowbd, high=act_highbd, dtype=np.float32)
       
        obs_lowbd  = np.ones(12,dtype=np.float32)  * -100
        obs_highbd = np.ones(12,dtype=np.float32)  *  100
        self.observation_space = spaces.Box(low=obs_lowbd , high=obs_highbd, dtype=np.float64)
        self.target_height = 0.2*math.cos(math.pi/6) + 0.22*math.cos(math.pi/6-math.pi/3) + 0 + 0.3/2


    def step(self, action):
        obs = self.w.step(action)
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

    def close(self):
        self.w.close()

    def seed(self, seed=None):
        pass


