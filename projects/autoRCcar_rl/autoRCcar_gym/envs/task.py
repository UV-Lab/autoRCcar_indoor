import math
import numpy as np
import random

class Task:

    def generate_map(self):
        ## goal point
        rand_x = int(random.choice(['1', '-1']))*random.randint(7, 9)
        rand_y = int(random.choice(['1', '-1']))*random.randint(7, 9)
        dist = np.sqrt(rand_x*rand_x + rand_y*rand_y)

        goal_err_x = rand_x - self.init_pos[0]
        goal_err_y = rand_y - self.init_pos[1]        

        ## obstacle
        ob_x = goal_err_x/2 + random.randint(-5,5)*0.1
        ob_y = goal_err_y/2 + random.randint(-5,5)*0.1
        ob_r = 1 + random.randint(0,5)*0.1

        return [rand_x, rand_y, dist], [ob_x, ob_y, ob_r]


    def calc_delta_angle(current, target, heading):
        
        del_x = target[0] - current[0]
        del_y = target[1] - current[1]

        angle_rad = math.atan2(del_y, del_x)

        if angle_rad <= 0:  # -pi~pi >> 0~2pi
            angle_rad += 2*np.pi

        if angle_rad > heading:
            delta_angle = angle_rad - heading
        else:
            delta_angle = heading - angle_rad

        if delta_angle >= np.pi:
            delta_angle -= 2*np.pi

        # return delta_angle
        return np.abs(delta_angle)
    
    def is_goal(self):
        current = self.state[0:2]
        goal = self.goal['position']

        del_x = goal[0] - current [0]
        del_y = goal[1] - current [1]
        delta_distance = np.sqrt(del_x*del_x + del_y*del_y)

        if delta_distance <= self.goal['radius']:
            return True
        else:
            return False

    
    def is_collision(self):
        current = self.state[0:2]
        obstacle = self.obstacle['position']

        del_x = obstacle[0] - current [0]
        del_y = obstacle[1] - current [1]
        delta_distance = np.sqrt(del_x*del_x + del_y*del_y)

        if delta_distance <= self.obstacle['radius']:
            return True
        else:
            return False