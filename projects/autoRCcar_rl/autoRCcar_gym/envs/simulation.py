import math
import numpy as np

class SimCar:
    # def init_sim(self):


    def UpdatebicycleModel(self, action):

        l = self.wheelbase
        dt = 1/self.simulation_hz

        ## action [steering, speed]
        u_delta = action[0]
        u_v = action[1]

        ## state [x, y, yaw. yaw_rate, speed]
        x = self.state[0]
        y = self.state[1]
        yaw = self.state[2]
        yaw_rate = self.state[3]
        speed = self.state[4]

        yaw_rate = (speed / l) * math.tan(u_delta)
        speed = speed * 0.1 + u_v * (1 - 0.1)
        
        x = x + speed * math.cos(yaw) * dt
        y = y + speed * math.sin(yaw) * dt
        yaw = yaw + yaw_rate * dt


        ## -pi ~ pi
        # if yaw > np.pi:
        #     yaw = yaw - 2*np.pi
        # elif yaw < -np.pi:
        #     yaw = 2*np.pi + yaw

        ## 0 ~ 2pi
        if yaw >= 2*np.pi:
            yaw = yaw - 2*np.pi
        elif yaw < 0:
            yaw = 2*np.pi + yaw

        state = np.array([x, y, yaw, yaw_rate, speed], dtype=np.float32)

        return state

        
