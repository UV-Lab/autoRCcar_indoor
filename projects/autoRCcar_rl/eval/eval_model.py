import autoRCcar_gym
import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import sys, ast
from stable_baselines3 import PPO
from sb3_contrib import TQC
import utils

fname = sys.argv[1]

if fname.split('_')[0] == 'best':
    task = fname.split('_')[1]
    model_name = fname+'/best_model'
else:
    task = fname.split('_')[0]
    model_name = fname

env = gym.make(task)

## Load model
if 'ppo' in fname:
    model = PPO.load(model_name, env=env)
elif 'tqc' in fname:
    model = TQC.load(model_name, env=env)
else:
    print("file name error")

## Set waypoint
wp = []
with open("waypoints.txt", 'r') as file:
    for line in file:
        line = line.strip()
        try:
            point = list(map(float, line.split()))
            rounded_point = [round(num, 1) for num in point]
            wp.append(rounded_point)
        except:
            pass
env.wp_end = len(wp)

## Rest env
wp_idx = 0
obs, info = env.reset(set_goal=wp[wp_idx])
print("\n\nWaypoint Index = ", wp_idx)
print(info['task'])
print("init_state : ", info['state'], "heading[deg] : ", np.degrees(info['state'][2]))
print("goal : ", info['goal'])
print("obstacle : ", info['obstacle'])
print("---------------------")
print("obs : ", obs)
wp_idx += 1

## Plot init env
plt.figure()
plt.title('Map'); plt.xlabel('X'); plt.ylabel('Y')
plt.axis('equal'); plt.grid(True)
utils.plot_init_env(info, wp)
utils.plot_obstacle(info)


## Record data
vheicle = {'t':[], 'x':[], 'y':[], 'speed':[], 'yaw':[], 'u_delta':[], 'u_v':[]}
dat = {'reward':[], 'dist_goal':[], 'dist_ob':[], 'angle_goal':[], 'angle_ob':[]}

## Run model
loop = 0
total_reward = 0
episode_reset = False
while True:

    if episode_reset:
        obs, info = env.reset(set_continue=True, set_goal=wp[wp_idx])
        utils.plot_obstacle(info)

        print("\n\nWaypoint Index = ", wp_idx, loop)
        print("init_state : ", info['state'], "heading[deg] : ", np.degrees(info['state'][2]))
        print("goal : ", info['goal'])
        print("obstacle : ", info['obstacle'])
        print("---------------------")
        print("obs : ", obs, "\n\n")
        plt.scatter(info['state'][0], info['state'][1], color='k', s=35)
        wp_idx += 1
        episode_reset = False

    action, _states = model.predict(obs, deterministic=True)
    vheicle['u_delta'].append(np.degrees(action[0]))
    vheicle['u_v'].append(action[1])

    obs, reward, terminated, _, info = env.step(action)
    total_reward += reward

    vheicle['t'].append(loop)
    vheicle['x'].append(info['state'][0])
    vheicle['y'].append(info['state'][1])
    vheicle['yaw'].append(np.degrees(obs[7]))
    vheicle['speed'].append(obs[8])
    dat['reward'].append(reward)

    dat['dist_goal'].append(np.linalg.norm([obs[0], obs[1]]))
    dat['angle_goal'].append(np.degrees(obs[2]))
    dat['dist_ob'].append(np.linalg.norm([obs[3], obs[4]]))
    dat['angle_ob'].append(np.degrees(obs[5]))

    if terminated:
        if wp_idx < env.wp_end:
            episode_reset = True
            terminated = False
        else:
            break

    loop += 1 

print(f'\n{loop} th Done) Total Reward : {total_reward}')

## Result
plt.scatter(vheicle['x'], vheicle['y'], s=7, label='vehicle')
# plt.legend(loc='best')
plt.show()