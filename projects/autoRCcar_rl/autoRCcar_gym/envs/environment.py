import numpy as np
import random
import os

import gymnasium as gym
from gymnasium import spaces

from autoRCcar_gym.envs.simulation import SimCar
from autoRCcar_gym.envs.task import Task

class autoRCcarEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
    
    def __init__(self, render_mode = None):
        self.task = 'avoid-v0'
        self.map_size = 15  # [m] total_size = map_size *2
        self.steps_cnt = 0 
        self.max_step = 500

        ## Waypoint
        self.vehicle_heading = None
        self.wp_end = 0
        self.wp_now = 0
        
        ## Goal
        self.goal = {}
        self.goal_radius = 0.3
        self.goal_flag = False

        ## Obstacle
        self.obstacle = {}
        
        ## Vehicle
        self.state = None
        self.state_before = None
        self.simulation_hz = 10.0      # [Hz]
        self.wheelbase = 0.3           # [m]
        self.steering_bound = [-np.radians(45.0), np.radians(45.0)]  # [deg->rad]
        self.speed_bound = [0.0, 3.0]  # [m/s]
        self.init_pos = [0, 0]         # [m]
        
        self.obs_before = None

        ## Action space [steering, speed]
        act_low_bound = np.array([self.steering_bound[0], self.speed_bound[0]], dtype=np.float32)
        act_high_bound = np.array([self.steering_bound[1], self.speed_bound[1]], dtype=np.float32)
        self.action_space = spaces.Box(act_low_bound, act_high_bound, dtype=np.float32) 

        ## Observation space 
        obs_low_bound = np.array([0, 0, 0, \
                                  0, 0, 0, 1, \
                                  0, self.speed_bound[0]], dtype=np.float32)
        obs_high_bound = np.array([self.map_size, self.map_size, np.pi, \
                                   self.map_size, self.map_size, np.pi, 1.5, \
                                   2*np.pi, self.speed_bound[1]], dtype=np.float32) 
        self.observation_space = spaces.Box(obs_low_bound, obs_high_bound, dtype=np.float32)


    def step(self, action):
        self.steps_cnt += 1

        self.state = SimCar.UpdatebicycleModel(self, action)
        current_pos = self.state[0:2]

        ## Update goal
        goal_err_x = self.goal['position'][0] - current_pos[0]
        goal_err_y = self.goal['position'][1] - current_pos[1]
        goal_del_angle = Task.calc_delta_angle(current_pos, self.goal['position'], self.state[2])
        self.goal['del_angle'] = np.degrees(goal_del_angle)

        ## Update obstacle
        ob_err_x = self.obstacle['position'][0] - current_pos[0]
        ob_err_y = self.obstacle['position'][1] - current_pos[1]
        ob_del_angle = Task.calc_delta_angle(current_pos, self.obstacle['position'], self.state[2])
        self.obstacle['del_angle'] = np.degrees(ob_del_angle)

        ## Update observation
        observation = np.array([np.abs(goal_err_x), np.abs(goal_err_y), goal_del_angle, \
                                np.abs(ob_err_x), np.abs(ob_err_y), ob_del_angle, self.obstacle['radius'], \
                                self.state[2], self.state[4]], dtype=np.float32)        

        ## Check terminate
        collision = False
        goal = False

        collision = Task.is_collision(self)
        if not collision:
            goal = Task.is_goal(self)
        terminated = bool(
            np.abs(current_pos[0]-self.init_pos[0]) >= self.map_size
            or np.abs(current_pos[1]-self.init_pos[1]) >= self.map_size
            or self.steps_cnt == self.max_step
            or collision
        )

        ## Update reward
        reward = 0
        if terminated:
            reward = -150
        elif goal:
            reward = 200
            terminated = True
            self.goal_flag = True
        else:
            ## relative distance (vehicle-goal)
            current_distance = np.sqrt(goal_err_x*goal_err_x + goal_err_y*goal_err_y)
            before_distance = np.sqrt(self.obs_before[0]*self.obs_before[0] + self.obs_before[1]*self.obs_before[1])

            val = (-1/np.pi)*goal_del_angle + 1  # vehicle/goal relative angle (0 ~ 1)
            if current_distance < before_distance:
                reward += val
            else:
                reward -= 1

            val = (2/3)*self.state[4] + (-1)  # Speed reward (-1 ~ 1)
            reward += val

        ## information
        info = {}
        info['steps'] = self.steps_cnt
        info['state'] = self.state
        info['done'] = [terminated, collision, goal]

        self.state_before = self.state
        self.obs_before = observation

        return observation, reward, terminated, False, info


    def reset(self, seed=None, options=None, set_continue=None, set_goal=None):  # New episode
        super().reset(seed=seed)

        self.steps_cnt = 0 
        self.goal_flag = False    
        
        ## Initialize vehicle's state [x, y, yaw. yaw_rate, speed]
        if set_continue is None:
            random_heading_deg = random.randint(0, 359)  # 0 ~ 359 사이 1deg 간격 무작위 추출
            init_head_rad = np.radians(random_heading_deg)
            self.state = np.array([self.init_pos[0], self.init_pos[1], init_head_rad, 0, self.speed_bound[0]], dtype=np.float32)
        else:
            self.init_pos = [self.state[0], self.state[1]]
            self.state = self.state

        if self.task == 'avoid-v0':
            ## Initialize map
            tmpGoal, tmpOb = Task.generate_map(self, set_goal)
        elif self.task == 'avoid-v1':
            ## Initialize map
            tmpGoal, tmpOb = Task.generate_general_map(self, set_goal)

        self.goal['position'] = [tmpGoal[0], tmpGoal[1]]
        self.goal['radius'] = self.goal_radius

        goal_err_x = self.goal['position'][0] - self.state[0]
        goal_err_y = self.goal['position'][1] - self.state[1]
        goal_del_angle = Task.calc_delta_angle(self.init_pos, self.goal['position'], self.state[2])
        self.goal['delta_angle'] = np.degrees(goal_del_angle)

        self.obstacle['position'] = [tmpOb[0]+self.state[0], tmpOb[1]+self.state[1]]
        self.obstacle['radius'] = tmpOb[2]
        ob_err_x = self.obstacle['position'][0] - self.state[0]
        ob_err_y = self.obstacle['position'][1] - self.state[1]
        ob_del_angle = Task.calc_delta_angle(self.state, self.obstacle['position'], self.state[2])
        self.obstacle['delta_angle'] = np.degrees(ob_del_angle)

        if tmpOb[0]==0 and tmpOb[1]==0 and tmpOb[2]==0:
            ob_del_angle = 0
            self.obstacle['delta_angle'] = np.degrees(ob_del_angle)

        ## Initialize observation
        observation = np.array([np.abs(goal_err_x), np.abs(goal_err_y), goal_del_angle, \
                                np.abs(ob_err_x), np.abs(ob_err_y), ob_del_angle, self.obstacle['radius'], \
                                self.state[2], self.state[4]], dtype=np.float32)

        self.obs_before = observation

        ## Initialize information
        info = {}
        info['task'] = self.task
        info['state'] = self.state
        info['goal'] = self.goal
        info['obstacle'] = self.obstacle

        return observation, info


    def render(self):
        ## TBD
        pass


class autoRCcarEnv_rev(autoRCcarEnv):

    def __init__(self, render_mode = None):
        self.task = 'avoid-v1'
        self.map_size = 15  # [m] total_size = map_size *2
        self.steps_cnt = 0 
        self.max_step = 500

        ## Waypoint
        self.vehicle_heading = None
        self.wp_end = 0
        self.wp_now = 0
        
        ## Goal
        self.goal = {}
        self.goal_radius = 0.3
        self.goal_flag = False

        ## Obstacle
        self.obstacle = {}
        
        ## Vehicle
        self.state = None
        self.state_before = None
        self.simulation_hz = 10.0      # [Hz]
        self.wheelbase = 0.3           # [m]
        self.steering_bound = [-np.radians(45.0), np.radians(45.0)]  # [deg->rad]
        self.speed_bound = [0.0, 3.0]  # [m/s]
        self.init_pos = [0, 0]         # [m]
        
        self.obs_before = None

        ## Action space [steering, speed]
        act_low_bound = np.array([self.steering_bound[0], self.speed_bound[0]], dtype=np.float32)
        act_high_bound = np.array([self.steering_bound[1], self.speed_bound[1]], dtype=np.float32)
        self.action_space = spaces.Box(act_low_bound, act_high_bound, dtype=np.float32) 

        ## Observation space 
        obs_low_bound = np.array([0, 0, 0, \
                                  0, 0, 0, 0, \
                                  0, self.speed_bound[0]], dtype=np.float32)
        obs_high_bound = np.array([self.map_size, self.map_size, np.pi, \
                                   self.map_size, self.map_size, np.pi, 5, \
                                   2*np.pi, self.speed_bound[1]], dtype=np.float32) 
        self.observation_space = spaces.Box(obs_low_bound, obs_high_bound, dtype=np.float32)