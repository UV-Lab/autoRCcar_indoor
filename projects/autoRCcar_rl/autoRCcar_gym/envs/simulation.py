import math
import numpy as np

class SimCar:
    # def init_sim(self):


    def UpdatebicycleModel(self, action):

        l = self.wheelbase
        dt = 1/self.simulation_hz

        u_delta = action[0]
        u_v = action[1]
        x = self.state[0]
        y = self.state[1]
        speed = self.state[2]
        yaw = self.state[3]
        yaw_rate = self.state[4]

        x = x + speed * math.cos(yaw) * dt
        y = y + speed * math.sin(yaw) * dt
        yaw = yaw + yaw_rate * dt
        yaw_rate = (speed / l) * math.tan(u_delta)
        speed = speed * 0.1 + u_v * (1 - 0.1)

        if yaw >= np.pi:
            yaw = 2*np.pi - yaw
        elif yaw <= -np.pi:
            yaw = 2*np.pi + yaw

        state = np.array([x, y, speed, yaw, yaw_rate], dtype=np.float32)

        return state

        
