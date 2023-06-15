import gymnasium as gym
from env.hopper2dEnv import Hopper2dEnv
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecExtractDictObs, VecMonitor

gym.envs.register(
        id='Hopper2dEnv-v0',
        entry_point='env:Hopper2dEnv',
    )

def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """

    def _init():
        env = gym.make(env_id)
        # use a seed for reproducibility
        # Important: use a different seed for each environment
        # otherwise they would generate the same experiences
        env.reset(seed=seed + rank)
        return env

    set_random_seed(seed)
    return _init

if __name__ == "__main__":
    env = gym.make('Hopper2dEnv-v0')
    obs = env.reset()
    breakpoint()
    num_cpu = 6  # Number of processes to use
    env_id = 'Loco3dEnv-v0'
    checkpoint_callback = CheckpointCallback(
      save_freq=10000,
      save_path="./logs/",
      name_prefix="rl_model"
    )
    # vec_env = make_vec_env('Loco3dEnv-v0', n_envs=num_cpu, vec_env_cls = SubprocVecEnv())
    envs = [make_env(env_id, i + num_cpu) for i in range(num_cpu)]
    train_env = SubprocVecEnv(envs
        ,
        start_method="forkserver",
    )
    train_env = VecMonitor(train_env)


    # vec_env = VecMonitor(vec_env)
    model = PPO("MlpPolicy", train_env, verbose=1, tensorboard_log="./loco3d_linear_vision/")
    model.learn(total_timesteps=5000000)
    model.save("loco3d_linear_vision")
