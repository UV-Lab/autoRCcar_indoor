import gym
import os
import autoRCcar_gym
import numpy as np
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
import autoRCcar_gym.envs.utils as util
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

obs, info = env.reset()
print(f"Model : {model_name}, Task : {env.task}")
print(f"init state : {info['state']}")
print(f"init obs : {obs}")
print(f"goal : {info['goal']}, init_heading : {info['init_heading']}")

result = {'t':[], 'x':[], 'y':[], 'speed':[], 'yaw':[],
          'u_delta':[], 'u_v':[], 'reward':[],
          'err_offset':[], 'err_dist':[], 'err_yaw':[],
          'obs_err':[]}

loop = 0
total_reward = 0
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, dones, _, info = env.step(action)

    total_reward += reward
    
    ## data record
    result = util.data_record(loop, result, obs, action, reward, info)

    if dones:
        done = info['done']
        print(f'\n{loop} th) Goal : {done[0]}, Terminated : {done[1]}, Reward : {total_reward}')
        print(f'end obs : {obs}\n')
        break

    loop += 1 



fig = plt.figure(figsize=(30,17))

util.state_plot(result, info)
util.data_plot(result, info)


plt.show()
