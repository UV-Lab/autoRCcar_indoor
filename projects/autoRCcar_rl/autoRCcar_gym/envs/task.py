import math
import numpy as np
import random

class Task:

    def generate_map(self, set_goal):
        
        ## goal point
        if set_goal is None:
            rand_x = int(random.choice(['1', '-1']))*random.randint(7, 9)
            rand_y = int(random.choice(['1', '-1']))*random.randint(7, 9)
        else:
            rand_x = set_goal[0]
            rand_y = set_goal[1]

        dist = np.sqrt(rand_x*rand_x + rand_y*rand_y)

        goal_err_x = rand_x - self.state[0]
        goal_err_y = rand_y - self.state[1]

        ## obstacle
        ob_x = goal_err_x/2 + random.randint(-5,5)*0.1
        ob_y = goal_err_y/2 + random.randint(-5,5)*0.1
        ob_r = 1 + random.randint(0,5)*0.1

        return [rand_x, rand_y, dist], [ob_x, ob_y, ob_r]


    def generate_general_map(self, set_goal):

        env_case = random.randint(1,2) # None, Obstacle(fixed)
        
        ## goal point
        if set_goal is None:
            rx = random.randint(2, 10)
            ry = random.randint(2, 10)
            rand_x = int(random.choice(['1', '-1']))*rx
            rand_y = int(random.choice(['1', '-1']))*ry
        else:
            rand_x = set_goal[0]
            rand_y = set_goal[1]

        dist = np.sqrt(rand_x*rand_x + rand_y*rand_y)

        goal_err_x = rand_x - self.state[0]
        goal_err_y = rand_y - self.state[1]

        goal_dist = np.sqrt(goal_err_x*goal_err_x + goal_err_y*goal_err_y)

        if env_case == 1:
            ob_x = 0
            ob_y = 0
            ob_r = 0
        else:
            if goal_dist < 4:
                ob_x = goal_err_x/2
                ob_y = goal_err_y/2
                ob_r = 1 + random.randint(0,5)*0.1
            else:
                ## obstacle
                ob_x = goal_err_x/2 + random.randint(-5,5)*0.1
                ob_y = goal_err_y/2 + random.randint(-5,5)*0.1
                
                radius = random.randint(2, max(2, int(goal_dist/3)))
                ob_r = radius + random.randint(0,9)*0.1

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