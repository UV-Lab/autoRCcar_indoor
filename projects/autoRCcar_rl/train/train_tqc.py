import gym
import os, time
import autoRCcar_gym
import numpy as np
from stable_baselines3.common.callbacks import EvalCallback
from sb3_contrib import TQC

ver = 'tqc1'
# task = 'autoRCcar_straight'
# task = 'autoRCcar_waypoint'
task = 'autoRCcar_avoid'
env = gym.make(task)

task_name = task.split('_')[1]
best_model = 'model/best_' + task_name + '_' + ver
final_model = 'model/' + task_name + '_' + ver

log_path = os.path.join('log')

policy_kwargs = dict(n_critics=2, n_quantiles=25)
model = TQC("MlpPolicy", env, top_quantiles_to_drop_per_net=2, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log=log_path)
tqc_path = os.path.join(best_model)
eval_env = model.get_env()
eval_callback = EvalCallback(eval_env=eval_env, best_model_save_path=tqc_path,
                             n_eval_episodes=10,
                             eval_freq=200000, verbose=1,
                             deterministic=True, render=False)

model.learn(total_timesteps=5000000, log_interval=100, callback=eval_callback)

tqc_path = os.path.join(final_model)
model.save(tqc_path)
