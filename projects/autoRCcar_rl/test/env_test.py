import gym
import autoRCcar_gym
import numpy as np
import matplotlib.pyplot as plt

task = 'autoRCcar_straight'
env = gym.make(task)

obs, info = env.reset()
print("init state : ", info['state'])
print("init obs : ", obs)

ref_x = np.linspace(-11, 11, 100)
ref_y = np.tan(info['state'][3]) * ref_x

result = {'t':[], 'x':[], 'y':[], 'yaw':[], 'steer':[], 'speed':[], 'err_offset':[], 'err_dist':[], 'err_yaw':[]}
total_reward = 0
for i in range(5000):

    # if i >= 500 and i < 1000:
    #     action = np.array([0.5, 0.5], dtype=np.float32)
    # elif i >= 1000 and i < 2000:
    #     action = np.array([-0.5, 0.5], dtype=np.float32)
    # else:
    #     action = np.array([0, 0.5], dtype=np.float32)

    action = env.action_space.sample()
    action = [-0.3, 0.3]

    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward

    ## data save
    result['t'].append(i)
    result['x'].append(info['state'][0])
    result['y'].append(info['state'][1])
    result['yaw'].append(np.degrees(info['state'][3]))
    result['steer'].append(np.degrees(action[0]))
    result['speed'].append(action[1])
    result['err_dist'].append(obs[0])
    result['err_offset'].append(obs[1])
    result['err_yaw'].append(obs[3])


    if terminated or truncated:
        print("Terminated : ", i)
        break


print(f'{i} th, Reward : {total_reward}, Error : {obs}')



fig = plt.figure(figsize=(18,10))
plt.subplot(2,2,1)
plt.fill_between(ref_x, ref_y+0.3, ref_y-0.3, color='y', alpha=0.3, label='safety zone')
plt.scatter(result['x'], result['y'], label='vehicle')
plt.scatter(result['x'][0], result['y'][0], color='r', marker='*', s=20, label='start')
# plt.plot(ref_x, ref_y, color='k', label='reference')
plt.title('Vehicle Position')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(-9,9)
plt.ylim(-9,9)
plt.legend(loc='best')
plt.grid(True)


ax1 = plt.subplot(2,2,2)
ax1.set_xlabel('Epoch')
ax1.set_ylabel('Steering angle [deg]')
ax1.plot(result['t'], result['steer'], color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax2 = ax1.twinx()
ax2.set_ylabel('Speed')
ax2.plot(result['t'], result['speed'], color='tab:red')
ax2.tick_params(axis='y', labelcolor='tab:red')
plt.title('Action')
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(result['t'], result['err_offset'])
plt.title('Terminate')
plt.xlabel('Epoch')
plt.ylabel('Offset Error')
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(result['t'], result['err_dist'])
plt.title('Terminate')
plt.xlabel('Epoch')
plt.ylabel('Distance Error')
plt.grid(True)

plt.show()