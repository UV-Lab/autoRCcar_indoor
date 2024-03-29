import numpy as np

class Rewards:

    def task_straight(self, terminated, goal):
        reward = 0
        if terminated:
            reward -= 500
        elif goal:
            reward += 1000
            terminated = True
        else:
            if self.state[2] < 0.1:
                reward -= 5
            else:
                reward += round(self.state[2]*10,0)*0.5

        return reward, terminated
    
    
    def task_waypoint(self, terminated, goal, err):
        reward = 0
        # gain = (self.max_step - self.steps_cnt)/self.max_step + 1
        gain = 1.5 - 0.5 * self.steps_cnt / self.max_step
        if terminated:
            reward = -1000
        elif goal:
            reward = 1000 *gain
            terminated = True
        else:
            reward = -self.steps_cnt/self.max_step

        if self.target_error_before > err:
            reward += 1
        else:
            reward -= 1

        if self.state[2] < 0.1:
            if reward > 0:
                reward *= 0.5
            else:
                reward *= 1.5

        if self.steps_cnt == 1:
            reward = 0

        return reward, terminated

    def task_avoid(self, err):
        goal = self.info['done'][0]
        terminated = self.info['done'][1]
        collison = self.info['done'][2]

        reward = 0         
        if self.target_error_before > err:
            reward += 1
        elif self.obs[6] < 1.0:
            reward -= 2
        else:
            reward = -self.steps_cnt/self.max_step

        if self.state[2] < 0.1:
            if reward > 0:
                reward *= 0.5
            else:
                reward *= 1.5
        
        gain = 1.5 - 0.5 * self.steps_cnt / self.max_step
        if terminated:
            reward = -500 *gain
        elif goal:
            reward = 500 *gain
            terminated = True
        elif collison:
            reward -= 100
            terminated = True

        if self.steps_cnt == 1:
            reward = 0



        return reward, terminated