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
    # env = gym.make('Hopper2dEnv-v0')
    # obs = env.reset()
    # breakpoint()
    # count = 0
    # while True:
    #     action = np.array([0.0, 0.0, 0.0])
    #     env.step(action)
    #     env.render()
    #     count += 1
    #     if count > 1000:
    #         break

    LOG_PATH = "./logs/jumpJun22"
    num_cpu = 12  # Number of processes to use
    env = SubprocVecEnv([make_env('Hopper2dEnv-v0', i) for i in range(num_cpu)])
    env = VecMonitor(env)
    checkpoint_callback = CheckpointCallback(
      save_freq=40000,
      save_path=LOG_PATH,
      name_prefix="jump"
    )

    eval_callback = EvalCallback(
        env, 
        best_model_save_path=LOG_PATH,
        log_path=LOG_PATH,
        eval_freq=10000,
    )


    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=LOG_PATH)

    model.learn(total_timesteps=3000000, callback=[checkpoint_callback, eval_callback])
    model.save(LOG_PATH + "/model")
