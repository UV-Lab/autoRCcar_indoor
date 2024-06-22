import autoRCcar_gym
import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import sys
from stable_baselines3 import PPO
from sb3_contrib import TQC

fname = sys.argv[1]

if fname.split('_')[0] == 'best':
    task = fname.split('_')[1]
    model_name = 'model/' + fname+'/best_model'
else:
    task = fname.split('_')[0]
    model_name = 'model/' + fname

env = gym.make(task)

## Load model
if 'ppo' in fname:
    model = PPO.load(model_name, env=env)
elif 'tqc' in fname:
    model = TQC.load(model_name, env=env)
   
## Rest env
obs, info = env.reset()
print(info['task'])
print("init_state : ", info['state'], "heading[deg] : ", np.degrees(info['state'][2]))
print("goal : ", info['goal'])
print("obstacle : ", info['obstacle'])
print("---------------------")
print("obs : ", obs)

## Plot init env
theta = np.linspace(0, 2*np.pi, 100)
ob_radius = info['obstacle']['radius']
obx = info['obstacle']['position'][0] + ob_radius * np.cos(theta)
oby = info['obstacle']['position'][1] + ob_radius * np.sin(theta)

goal_radius = info['goal']['radius']
gx = info['goal']['position'][0] + goal_radius * np.cos(theta)
gy = info['goal']['position'][1] + goal_radius * np.sin(theta)

yaw = info['state'][2]
hdx = info['state'][0] + 0.5*np.cos(yaw)
hdy = info['state'][1] + 0.5*np.sin(yaw)

plt.figure()
plt.subplot(2,3,1)
plt.scatter(info['state'][0], info['state'][1], color='b', marker='*', s=50,  label='Vehicle')
plt.plot([info['state'][0], hdx], [info['state'][1], hdy])
plt.scatter(info['goal']['position'][0], info['goal']['position'][1], color='m', marker='*', s=50,  label='Goal')
plt.plot(gx, gy)
plt.scatter(info['obstacle']['position'][0], info['obstacle']['position'][1], color='r', s=30,  label='Obstacle')
plt.plot(obx, oby)
plt.title('Map')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.legend(loc='best')
plt.grid(True)


## Record data
vheicle = {'t':[], 'x':[], 'y':[], 'speed':[], 'yaw':[], 'u_delta':[], 'u_v':[]}
dat = {'reward':[], 'dist_goal':[], 'dist_ob':[], 'angle_goal':[], 'angle_ob':[]}

## Run model
loop = 0
total_reward = 0
while True:
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
        break

    loop += 1 

print(f'\n{loop} th Done) Total Reward : {total_reward}')

## Result
plt.subplot(2,3,1)
plt.scatter(vheicle['x'], vheicle['y'], s=7, label='vehicle')

ax1 = plt.subplot(2, 3, 2)
ax1.plot(vheicle['t'], vheicle['yaw'], color='tab:blue', label='Yaw [deg]')
ax1.plot(vheicle['t'], vheicle['u_delta'], color='tab:red', label='Steering [deg]')
ax1.set_ylabel('Heading [deg]', color='tab:blue')
ax1.set_xlabel('Epoch')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_ylim(-50, 360)
ax1.legend(loc='best')
plt.title('Heading')
plt.grid(True)

ax1 = plt.subplot(2, 3, 3)
ax1.plot(vheicle['t'], vheicle['speed'], color='tab:blue', label='Speed [m/s]')
ax1.plot(vheicle['t'], vheicle['u_v'], color='tab:red', label='Accel [m/s]')
ax1.set_ylabel('Speed [m/s]', color='tab:blue')
ax1.set_xlabel('Epoch')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_ylim(0, 3.5)
ax1.legend(loc='best')
plt.title('Speed')
plt.grid(True)



ax1 = plt.subplot(2,3,4)
ax1.plot(vheicle['t'], dat['dist_goal'], label='goal [m]')
ax1.plot(vheicle['t'], dat['dist_ob'], label='obstacle [m]')
ax1.set_ylabel('Angle [deg]')
ax1.set_xlabel('Epoch')
ax1.tick_params(axis='y')
# ax1.set_ylim(0, 10)
ax1.legend(loc='best')
plt.title('Angle Error')
plt.grid(True)


ax1 = plt.subplot(2,3,5)
ax1.plot(vheicle['t'], dat['angle_goal'], label='goal [deg]')
ax1.plot(vheicle['t'], dat['angle_ob'], label='obstacle [deg]')
ax1.set_ylabel('Distance [m]')
ax1.set_xlabel('Epoch')
ax1.tick_params(axis='y')
# ax1.set_ylim(0, 10)
ax1.legend(loc='best')
plt.title('Distance Error')
plt.grid(True)


ax1 = plt.subplot(2,3,6)
ax1.set_xlabel('Epoch')
ax1.set_ylabel('Reward')
ax1.scatter(vheicle['t'][:-1], dat['reward'][:-1])
ax2 = ax1.twinx()
ax2.scatter(vheicle['t'][-1], dat['reward'][-1], color='tab:red')
ax2.tick_params(axis='y', labelcolor='tab:red')
plt.title('Reward')
plt.grid(True)
plt.text(vheicle['t'][2], dat['reward'][-1], f'{fname}\nTotal reward : {round(total_reward,3)}', color='tab:red')

plt.show()