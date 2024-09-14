import matplotlib.pyplot as plt
import numpy as np

def plot_init_env(info, wp):
    ## Initial position
    yaw = info['state'][2]
    hdx = info['state'][0] + 0.5*np.cos(yaw)
    hdy = info['state'][1] + 0.5*np.sin(yaw)

    plt.scatter(info['state'][0], info['state'][1], color='b', marker='*', s=50,  label='Vehicle')
    plt.plot([info['state'][0], hdx], [info['state'][1], hdy])

    ## Waypoint
    theta = np.linspace(0, 2*np.pi, 100)
    goal_radius = info['goal']['radius']

    for idx, item in enumerate(wp):
        gx = item[0] + goal_radius * np.cos(theta)
        gy = item[1] + goal_radius * np.sin(theta)
        plt.scatter(item[0], item[1], color='m', marker='*', s=50,  label='Waypoint')
        if idx == len(wp) - 1:
            plt.plot(gx, gy, color='k')
        else:
            plt.plot(gx, gy, color='m')



def plot_obstacle(info, index=0, origin=None):
    theta = np.linspace(0, 2*np.pi, 100)
    ob_radius = info['obstacle']['radius']

    if index == 0:
        origin = [0, 0]

    obstacle_x = info['obstacle']['position'][0] + origin[0]
    obstacle_y = info['obstacle']['position'][1] + origin[1]

    obx = obstacle_x + ob_radius * np.cos(theta)
    oby = obstacle_y + ob_radius * np.sin(theta)

    plt.scatter(obstacle_x, obstacle_y, color='r', s=30,  label='Obstacle')
    plt.plot(obx, oby)
