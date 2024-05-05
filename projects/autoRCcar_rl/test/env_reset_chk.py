import autoRCcar_gym
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np

env = gym.make("avoid-v0")
observation, info = env.reset()

print(info['task'])
print("init_state : ", info['state'], "heading[deg] : ", np.degrees(info['state'][2]))
print("goal : ", info['goal'])
print("obstacle : ", info['obstacle'])
print("---------------------")
print("obs : ", observation)

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

plt.show()