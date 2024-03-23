import gym
import autoRCcar_gym
import numpy as np
import matplotlib.pyplot as plt

task = 'autoRCcar_waypoint'
env = gym.make(task)

obs, info = env.reset()
print("init : ", obs, info['goal'], info['init_heading'])


result = {'t':[], 'x':[], 'y':[], 'spd':[], 'heading':[], 'yaw':[], 'angle':[], 'steer':[], 'speed':[], 'delta_distance':[], 'delta_angle':[], 'reward':[]}
total_reward = 0
for i in range(50000):

    if i >= 250:
        action = np.array([-0, 0.9], dtype=np.float32)
    # elif i >= 1000 and i < 2000:
    #     action = np.array([-0.5, 0.5], dtype=np.float32)
    else:
        action = np.array([0, 0.5], dtype=np.float32)

    # action = env.action_space.sample()

    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward


    ## data save
    result['t'].append(i)
    result['x'].append(obs[0])
    result['y'].append(obs[1])
    result['spd'].append(obs[2])
    result['heading'].append(np.degrees(obs[3]))
    result['yaw'].append(np.degrees(info['state'][3]))
    result['delta_distance'].append(obs[4])
    result['delta_angle'].append(np.degrees(obs[5]))
    result['steer'].append(action[0])
    result['speed'].append(action[1])
    result['reward'].append(reward)
    

    if terminated or truncated:
        done = info['done']
        print(f"Terminated : {i}", done)
        print(obs)
        break

# offset = info['offset_error']
# delta = info['delta_distance']
print(f'{i} th, Reward : {total_reward}')


fig = plt.figure(figsize=(20,12))
plt.subplot(2,3,1)
plt.scatter(info['goal'][0], info['goal'][1], color='r', marker='*', s=200,  label='Goal')
plt.scatter(result['x'], result['y'], label='vehicle')
plt.scatter(result['x'][0], result['y'][0], color='m', marker='d', s=200, label='start')
plt.axis('equal')
plt.title('Vehicle Position')
plt.xlabel('X')
plt.ylabel('Y')
# plt.axis([-15, 15, -15, 15])
plt.legend(loc='best')
plt.grid(True)


plt.subplot(2,3,2)
plt.plot(result['t'], result['steer'], label='steering')
plt.plot(result['t'], result['speed'], label='speed(action)')
plt.plot(result['t'], result['spd'], label='speed')
plt.title('Action')
plt.xlabel('Epoch')
plt.ylabel('Action')
plt.ylim(-1.1, 1.1)
plt.legend()
plt.grid(True)

plt.subplot(2,3,3)
plt.plot(result['t'], result['heading'], label='heading')
plt.plot(result['t'], result['yaw'], label='yaw')
plt.title('Vehicle Heading')
plt.xlabel('Epoch')
plt.ylabel('Heading [deg]')
plt.legend()
plt.grid(True)

plt.subplot(2,3,4)
plt.plot(result['t'], result['delta_distance'])
plt.xlabel('Epoch')
plt.ylabel('Distance Error [m]')
plt.grid(True)

plt.subplot(2,3,5)
plt.plot(result['t'], result['delta_angle'])
plt.xlabel('Epoch')
plt.ylabel('Angle Error [rad]')
plt.grid(True)

plt.subplot(2,3,6)
plt.scatter(result['t'][:-1], result['reward'][:-1])
plt.title('Reward')
plt.xlabel('Epoch')
plt.ylabel('Reward')
plt.grid(True)

plt.show()