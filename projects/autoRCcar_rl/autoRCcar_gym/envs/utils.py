import math
import numpy as np
import matplotlib.pyplot as plt

def deg2rad(val):
    return val*(np.pi/180)

def add_noise(self, action, val):
    noise_mean = [0, 0]  # 각 요소에 대한 평균
    noise_std_dev = [np.radians(1*val), 0.1*val]  # [1deg]
    noisy_action = [a + np.random.normal(mean, std_dev) for a, mean, std_dev in zip(action, noise_mean, noise_std_dev)]
    
    if noisy_action[0] >= self.steering_bound[1]:
        noisy_action[0] = self.steering_bound[1]
    if noisy_action[0] <= self.steering_bound[0]:
        noisy_action[0] = self.steering_bound[0]
    if noisy_action[1] >= self.speed_bound[1]:
        noisy_action[1] = self.speed_bound[1]
    if noisy_action[1] <= self.speed_bound[0]:
        noisy_action[1] = self.speed_bound[0]
    
    return noisy_action

def denormalize(norm_data, min_val, max_val):
    denormalized_data = norm_data * (max_val - min_val) + min_val
    return denormalized_data

def state_plot(dat, info):

    plt.subplot(2,3,1)
    if info['task'] == 'straight':
        init_heading = np.radians(dat['yaw'][0])
        ref_x = np.linspace(-11, 11, 100)
        ref_y = np.tan(init_heading) * ref_x
        ref_y_bound = 0.3*np.sqrt(np.tan(init_heading)**2 + 1)
        plt.fill_between(ref_x, ref_y+ref_y_bound, ref_y-ref_y_bound, color='y', alpha=0.3, label='safety zone')

        if np.abs(dat['err_offset'][-1]) > 0.3:
            plt.scatter(dat['x'][-1], dat['y'][-1], color='r', marker='x', s=200, label='terminated')
    
    if info['task'] == 'avoid':
        theta = np.linspace(0, 2*np.pi, 100)
        radius = info['obstacle'][1]
        obx = info['obstacle'][0][0] + radius * np.cos(theta)
        oby = info['obstacle'][0][1] + radius * np.sin(theta)
        plt.scatter(info['obstacle'][0][0], info['obstacle'][0][1], color='m', s=100,  label='Obstacle')
        plt.plot(obx, oby)

    # elif info['task'] == 'waypoint':
    plt.scatter(dat['x'], dat['y'], s=7, label='vehicle')
    plt.scatter(dat['x'][0], dat['y'][0], color='b', marker='d', s=80, label='start')
    plt.scatter(info['goal'][0], info['goal'][1], color='r', marker='*', s=150, label='goal')
    plt.title('Vehicle Trajectory')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.legend(loc='best')
    plt.grid(True)

    ax1 = plt.subplot(2,3,2)
    ax1.set_ylabel('Heading [deg]', color='tab:blue')
    ax1.plot(dat['t'], dat['yaw'], color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    # ax1.set_ylim(0, 360)
    ax2 = ax1.twinx()
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('Steering Angle [deg] (-45~45)', color='tab:red')
    ax2.plot(dat['t'], dat['u_delta'], color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    # ax2.set_ylim(-45, 45)
    plt.title('Heading')
    plt.grid(True)

    ax1 = plt.subplot(2,3,3)
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Speed [m/s]', color='tab:blue')
    ax1.plot(dat['t'], dat['speed'], color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    # ax1.set_ylim(0, 1.1)
    ax2 = ax1.twinx()
    ax2.set_ylabel('Action Speed (0~1)', color='tab:red')
    ax2.plot(dat['t'], dat['u_v'], color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    # ax2.set_ylim(0, 1.1)
    plt.title('Speed')
    plt.grid(True)


def data_plot(dat, info):
    plt.subplot(2,3,4)
    plt.plot(dat['t'], dat['err_dist'])
    plt.xlabel('Epoch')
    plt.ylabel('Error of Distance [m]')
    plt.grid(True)


    plt.subplot(2,3,5)
    if info['task'] == 'straight':    
        plt.plot(dat['t'], dat['err_offset'])
        plt.xlabel('Epoch')
        plt.ylabel('Error of Offset [m]')
        plt.grid(True)
    elif info['task'] == 'waypoint':
        plt.plot(dat['t'], dat['err_yaw'])
        plt.xlabel('Epoch')
        plt.ylabel('Angle Error [deg]')
        plt.grid(True)
    elif info['task'] == 'avoid':
        plt.plot(dat['t'], dat['err_yaw'])
        plt.xlabel('Epoch')
        plt.ylabel('Angle Error [deg]')
        plt.grid(True)

    ax1 = plt.subplot(2,3,6)
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Reward')
    ax1.scatter(dat['t'][:-1], dat['reward'][:-1])
    ax2 = ax1.twinx()
    ax2.scatter(dat['t'][-1], dat['reward'][-1], color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    ax2.set_ylabel('Reward', color='tab:red')
    plt.title('Reward')
    plt.grid(True)


def data_record(i, result, obs, action, reward, info):
    if info['task'] == 'straight':
        yaw = np.degrees(info['state'][3])
    elif info['task'] == 'waypoint' or info['task'] == 'avoid':
        yaw = np.degrees(obs[3])

    # heading = yaw
    # if yaw < 0:
    #     heading = 360 + yaw
    # else:
    #     heading = yaw #% 360

    result['t'].append(i)

    result['u_delta'].append(np.degrees(action[0]))
    result['u_v'].append(action[1])

    result['reward'].append(reward)

    if info['task'] == 'straight':
        result['x'].append(info['state'][0])
        result['y'].append(info['state'][1])
        result['speed'].append(info['state'][2])
        result['yaw'].append(yaw)

        result['err_dist'].append(obs[0])
        result['err_offset'].append(obs[1])
        result['err_yaw'].append(obs[3])

    elif info['task'] == 'waypoint':
        result['x'].append(obs[0])
        result['y'].append(obs[1])
        result['speed'].append(obs[2])
        result['yaw'].append(yaw)
        
        result['err_dist'].append(obs[4])
        result['err_yaw'].append(np.degrees(obs[5]))

    elif info['task'] == 'avoid':
        result['x'].append(obs[0])
        result['y'].append(obs[1])
        result['speed'].append(obs[2])
        result['yaw'].append(yaw)
        
        result['err_dist'].append(obs[4])
        result['err_yaw'].append(np.degrees(obs[5]))
        result['obs_err'].append(obs[6])

    return result