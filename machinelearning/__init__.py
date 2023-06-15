from gym.envs.registration import register

register(
    id='LocoEnv-v0',
    entry_point='env:LocoEnv',
)