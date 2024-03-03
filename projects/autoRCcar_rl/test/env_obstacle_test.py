import gym
import autoRCcar_gym
import numpy as np
import matplotlib.pyplot as plt
from itertools import count
import termios, sys, os, tty


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

moveBindings = {
    '8': (0, 1),
    '2': (0, -1),
    '4': (-1, 0),
    '6': (1, 0),
}


settings = termios.tcgetattr(sys.stdin)

task = 'autoRCcar_avoid'
env = gym.make(task)

obs, info = env.reset()
print("init : ", obs, info['goal'], info['init_heading'])
print("obstacle : ", info['obstacle'])

result = {'t':[], 'x':[], 'y':[], 'spd':[], 'heading':[], 'yaw':[], 'angle':[], 'steer':[], 'speed':[], 'delta_distance':[], 'delta_angle':[], 'ob_error':[], 'reward':[]}
act = {'steering':0, 'speed':0}
total_reward = 0


# 그래프 초기화
xd1, yd1 = [], []
xd2, yd2 = [], []
xd3, yd3 = [], []
theta = np.linspace(0, 2*np.pi, 100)
radius = info['obstacle'][1]
obx = info['obstacle'][0][0] + radius * np.cos(theta)
oby = info['obstacle'][0][1] + radius * np.sin(theta)

# fig = plt.figure(figsize=(20,12))
# plt.subplot(131)
# plt.ion()
# fig, axs = plt.subplots(3)
# fig.tight_layout()
# plt.scatter(info['goal'][0], info['goal'][1], color='r', marker='*', s=200,  label='Goal')
# plt.scatter(info['obstacle'][0][0], info['obstacle'][0][1], color='m', s=100,  label='Obstacle')
# plt.plot(obx, oby)
# plt.title('Vehicle Position')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.axis('equal')
for i in count():

    ## Key 입력 
    key = getKey(settings)
    if key in moveBindings.keys():
        act['steering'] += np.radians(moveBindings[key][0]*5)
        act['speed'] += moveBindings[key][1]*0.1

        if act['steering'] >= env.steering_bound[1]:
            act['steering'] = env.steering_bound[1]
        elif act['steering'] <= env.steering_bound[0]:
            act['steering'] = env.steering_bound[0]
        if act['speed'] >= env.speed_bound[1]:
            act['speed'] = env.speed_bound[1]
        elif act['speed'] <= env.speed_bound[0]:
            act['speed'] = env.speed_bound[0]            
        action = np.array([act['steering'], act['speed']], dtype=np.float32)
    else:
        break

    obs2 = [f'{num:.2f}' for num in obs]
    yaw2 = round(np.degrees(obs[3]),2)
    print(f"\t{i} th, Action = {action}\tObs) speed:{obs2[2]}, yaw:{yaw2}, target_err:{obs2[5]}, ob_err:{obs2[6]}")

    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward


    ## data save
    result['t'].append(i)
    result['x'].append(obs[0])
    result['y'].append(obs[1])
    result['spd'].append(obs[2])
    result['heading'].append(np.degrees(obs[3]))
    result['yaw'].append(np.degrees(info['state'][3]))
    result['delta_distance'].append(obs[5])
    result['delta_angle'].append(np.degrees(obs[6]))
    result['ob_error'].append(obs[7])
    result['steer'].append(action[0])
    result['speed'].append(action[1])
    result['reward'].append(reward)

    if terminated or truncated:
        done = info['done']
        print(f"Terminated : {i}", done)
        print(obs)
        break


    # if i==0:
    #     plt.scatter(result['x'][0], result['y'][0], color='m', marker='d', s=200, label='start')
    # else:
    #     plt.scatter(result['x'], result['y'], color='b', label='vehicle')
    
    # plt.pause(0.1)
    # plt.show()
    


print(f'{i} th, Reward : {total_reward}')