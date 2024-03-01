import gym
import os
import autoRCcar_gym
import numpy as np
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
from tqdm import tqdm
import sys

ff = sys.argv[1]

if ff.split('_')[0] == 'best':
    task = 'autoRCcar_' + ff.split('_')[1]
    model_name = 'model/'+ff+'/best_model'
else:
    task = 'autoRCcar_' + ff.split('_')[0]
    model_name = 'model/'+ff

env = gym.make(task)
model = PPO.load(model_name, env=env)
print(f"Model : {model_name}, Task : {env.task}")

max_episode = 10000

# obs, info = env.reset()
# print("init state : ", info['state'])
# print("init obs : ", obs)


record_reward = []
record_step = []
record_task = []
for epi in tqdm(range(max_episode)):
    obs, info = env.reset()
    dones = False
    total_reward = 0
    for i in range(100000):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, _, info = env.step(action)

        total_reward += rewards

        if dones:
            break


    record_reward.append(total_reward)
    record_step.append(i)

    if env.task == 'straight':
        if dones and obs[0] < 0.3 and obs[1] < 0.3:
            record_task.append(True)
        else:
            record_task.append(False)
    elif env.task == 'waypoint': 
        if dones and obs[4] < 0.3:
            record_task.append(True)
        else:
            record_task.append(False)


ratio_T = record_task.count(True) / len(record_task)
ratio_F = record_task.count(False) / len(record_task)
print(f'Success : {ratio_T*100}% / {ratio_F*100}%')


true_indices = [index for index, value in enumerate(record_task) if value]
false_indices = [index for index, value in enumerate(record_task) if not value]


true_rewards = [record_reward[index] for index in true_indices]
average_reward = np.mean(true_rewards)

false_rewards = [record_reward[index] for index in false_indices]
average_false_reward = np.mean(false_rewards)

print(f'Mean reward : {round(average_reward,2)} / {round(average_false_reward,2)}')



true_step = [record_step[index] for index in true_indices]
average_step = np.mean(true_step)

false_step = [record_step[index] for index in false_indices]
average_false_step = np.mean(false_step)

print(f'Mean step : {round(average_step,2)} / {round(average_false_step,2)}')

