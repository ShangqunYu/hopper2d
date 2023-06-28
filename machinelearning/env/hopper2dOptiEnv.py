import os, sys
sys.path.append('../build/machinelearning')
from hopper2dOptiWrapper import Hopper2dOptiWrapper
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Optional, Tuple, Union
import math

class Hopper2dOptiEnv(gym.Env):
    def __init__(self):
        self.w = Hopper2dOptiWrapper()
        act_lowbd = np.ones(3,dtype=np.float32)  * 0.2
        act_highbd = np.ones(3,dtype=np.float32) *  1
        self.action_space = spaces.Box(low=act_lowbd, high=act_highbd, dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64)


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

    def seed(self, seed=None):
        pass


