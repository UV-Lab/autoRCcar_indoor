import gym
import os
import autoRCcar_gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

ver = 'v1'
# task = 'autoRCcar_straight'
# task = 'autoRCcar_waypoint'
task = 'autoRCcar_avoid'
env = gym.make(task)

task_name = task.split('_')[1]
best_model = 'model/best_' + task_name + '_' + ver
final_model = 'model/' + task_name + '_' + ver

log_path = os.path.join('log')
model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
ppo_path = os.path.join(best_model)
eval_env = model.get_env()

eval_callback = EvalCallback(eval_env=eval_env, best_model_save_path=ppo_path,
                             n_eval_episodes=10,
                             eval_freq=50000, verbose=1,
                             deterministic=True, render=False)
model.learn(total_timesteps=4000000, callback=eval_callback)
ppo_path = os.path.join(final_model)
model.save(ppo_path)
