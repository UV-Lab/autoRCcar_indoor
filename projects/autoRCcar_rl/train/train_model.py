import autoRCcar_gym
import gymnasium as gym
import os, sys
import numpy as np
from sb3_contrib import TQC
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

suffix = sys.argv[1]  # ex) ppo1, tqc2

num_eval = 10
task = 'avoid-v1'

env = gym.make(task)
best_model = 'model/best_' + task + '_' + suffix
final_model = 'model/' + task + '_' + suffix
log_path = os.path.join('log')
model_path = os.path.join(best_model)

if 'ppo' in suffix:
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
elif 'tqc' in suffix:
    policy_kwargs = dict(n_critics=2, n_quantiles=25)
    model = TQC("MlpPolicy", env, top_quantiles_to_drop_per_net=2, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log=log_path)    
else:
    print("Error - suffix!")
    sys.exit(0)

def evaluate_model(env, model, num_episodes):
    goal_arrival = 0
    for _ in range(num_episodes):
        obs = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            if done[0] and info[0]['done'][2]:  # Check if the episode is done and goal is reached
                goal_arrival += 1
    return goal_arrival

class CustomEvalCallback(EvalCallback):
    def __init__(self, eval_env, best_model_save_path, n_eval_episodes, verbose):
        super().__init__(eval_env=eval_env, best_model_save_path=best_model_save_path,
                         n_eval_episodes=n_eval_episodes, verbose=verbose)
        self.num_episodes = n_eval_episodes
    def _on_step(self) -> bool:
        total_goal_arrival = evaluate_model(self.eval_env, self.model, self.num_episodes)
        self.logger.record("eval/success_num", total_goal_arrival)
        return super()._on_step()

eval_env = model.get_env()

if 'ppo' in suffix:
    eval_callback = EvalCallback(eval_env=eval_env, best_model_save_path=model_path,
                                callback_after_eval=CustomEvalCallback(eval_env=eval_env, best_model_save_path=model_path, n_eval_episodes=num_eval, verbose=1),
                                n_eval_episodes=num_eval, eval_freq=100000, verbose=1,
                                deterministic=True, render=False)

elif 'tqc' in suffix:
    eval_callback = EvalCallback(eval_env=eval_env, best_model_save_path=model_path,
                                callback_after_eval=CustomEvalCallback(eval_env=eval_env, best_model_save_path=model_path, n_eval_episodes=num_eval, verbose=1), 
                                n_eval_episodes=num_eval, eval_freq=100000, verbose=1,
                                deterministic=True, render=False)

model.learn(total_timesteps=10000000, callback=eval_callback)
final_model_path = os.path.join(final_model)
model.save(final_model_path)