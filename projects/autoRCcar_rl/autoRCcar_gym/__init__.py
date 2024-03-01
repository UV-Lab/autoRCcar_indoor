from gym.envs.registration import register

register(
    id='autoRCcar_straight',
    entry_point='autoRCcar_gym.envs:autoRCcarEnv',
)

register(
    id='autoRCcar_waypoint',
    entry_point='autoRCcar_gym.envs:Follow_waypoints',
)