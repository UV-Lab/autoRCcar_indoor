from gymnasium.envs.registration import register

register(
     id="avoid-v0",
     entry_point="autoRCcar_gym.envs:autoRCcarEnv",
)