import gym
from gym import spaces
import numpy as np
import random
import math, time
from typing import Optional, Dict, Any
from autoRCcar_gym.envs.simulation import SimCar
from autoRCcar_gym.envs.task import Task
from autoRCcar_gym.envs.utils import deg2rad, add_noise
from autoRCcar_gym.envs.rewards import Rewards

class autoRCcarEnv(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 50,
    }
    def __init__(self, render_mode: Optional[str] = None):
        self.task = 'straight'
        ## Common variable
        self.init_position = [0, 0]
        self.init_speed = 0
        self.init_yaw_rate = 0
        self.heading_rad = 0
        self.state = None
        self.obs = None
        self.info = {}
        self.steps_cnt = None
        self.render_mode = render_mode

        ## 'autoRCcar_straight' task
        self.target_direction = 0
        self.target_distance = 10
        self.target_goal = [0, 0]
        self.target_threshold = 0.3

        # Vehicle data
        self.simulation_hz = 100                 # [Hz]
        self.steering_bound = [-deg2rad(45), deg2rad(45)]    # [deg->rad]
        self.speed_bound = [0, 1]
        self.wheelbase = 0.3                     # [m]

        # Action space
        action_low_bound = np.array([self.steering_bound[0], self.speed_bound[0]], dtype=np.float32)
        action_high_bound = np.array([self.steering_bound[1], self.speed_bound[1]], dtype=np.float32)
        self.action_space = spaces.Box(low=action_low_bound, high=action_high_bound)                 # steering_angle, speed

        # Observation space ('autoRCcar_straight' - Random)
        obs_low_bound = np.array([0, -self.target_threshold, self.speed_bound[0], 0], dtype=np.float32)
        obs_high_bound = np.array([self.target_distance, self.target_threshold, self.speed_bound[1], 2*np.pi], dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low_bound, high=obs_high_bound)  # error distance, error offset, speed, yaw_error



    def step(self, action):
        self.steps_cnt += 1

        noisy_action = add_noise(self, action, 1)
        self.state = SimCar.UpdatebicycleModel(self, noisy_action)
        # self.state = SimCar.UpdatebicycleModel(self, action)
        
        self.info['steps'] = self.steps_cnt
        self.info['state'] = self.state

        if self.task == 'straight':
            terminated, offset = Task._is_terminated_distance(self)
            goal, delta = Task._is_goal_distance(self)

            ## Observation
            delta_heading = np.abs(self.target_direction - self.state[3])
            self.obs = np.array([delta, offset, self.state[2], delta_heading], dtype=np.float32)
            self.info['done'] = [goal, terminated]

            ## Reward
            reward, terminated = Rewards.task_straight(self, terminated, goal)

        elif self.task == 'waypoint':
            goal, err = Task._is_goal_distance(self)
            angle = Task.calc_delta_heading(self)

            terminated = bool(
                # err > self.map_size +margin
                np.abs(self.state[0]) > self.map_size
                or np.abs(self.state[1]) > self.map_size
                or self.steps_cnt == self.max_step
            )

            ## Observation
            self.obs = np.array([self.state[0], self.state[1], self.state[2], self.state[3], err, angle], dtype=np.float32)
            self.info['done'] = [goal, terminated]
            # self.obs_norm = normalize_observation(self.obs, self.obs_low_bound, self.obs_high_bound)
            
            ## Reward
            reward, terminated = Rewards.task_waypoint(self, terminated, goal, err)

            self.target_error_before = err
            self.target_angle_before = angle

        elif self.task == 'avoid':
            # goal, err = Task._is_goal_distance(self)
            current = self.state[0:2]

            err_x = self.target_goal[0] - current[0]
            err_y = self.target_goal[1] - current[1]
            goal_error = np.sqrt(err_x*err_x + err_y*err_y)

            if np.abs(goal_error) < self.target_threshold:
                goal = True
            else:
                goal = False

            angle = Task.calc_delta_heading(self, self.target_goal)
            collison, self.ob_error, self.ob_angle = Task._is_collision(self)

            terminated = bool(
                np.abs(self.state[0]) > self.map_size
                or np.abs(self.state[1]) > self.map_size
                or self.steps_cnt == self.max_step
                # or collison
            )
            ## Observation
            self.obs = np.array([self.state[0],self.state[1],self.state[2],self.state[3], goal_error, angle, self.ob_error, self.ob_angle], dtype=np.float32)
            self.info['done'] = [goal, terminated, collison]
            self.obs_norm = normalize_observation(self.obs, self.obs_low_bound, self.obs_high_bound)

            ## Reward
            reward, terminated = Rewards.task_avoid(self, goal_error)

            self.target_error_before = goal_error
            self.info['obs'] = self.obs
            
        return self.obs_norm, reward, terminated, False, self.info


    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None, ):
        super().reset(seed=seed)

        self.steps_cnt = 0
        # random_heading_deg = random.randint(-17, 17) * 10  # -170 ~ 170 사이 10deg 간격 무작위 추출
        random_heading_deg = random.randint(0, 35) * 10  # 0 ~ 350 사이 10deg 간격 무작위 추출 (avoid)
        
        self.heading_rad = deg2rad(random_heading_deg)
        self.state = np.array([self.init_position[0], self.init_position[1], self.init_speed, self.heading_rad, self.init_yaw_rate], dtype=np.float32)

        if self.task == 'straight':
            init_target_distance = random.randint(4, self.target_distance)
            self.target_direction = self.heading_rad
            self.target_goal = np.array([np.cos(self.heading_rad), np.sin(self.heading_rad)], dtype=np.float32)
            self.target_goal *= init_target_distance  # Goal 4~10 meter

            self.obs = np.array([init_target_distance, 0, 0, 0], dtype=np.float32)

        elif self.task == 'waypoint':
            rand_x = int(random.choice(['1', '-1']))*random.randint(3,6)
            rand_y = int(random.choice(['1', '-1']))*random.randint(3,6)
            self.target_goal = [rand_x, rand_y]
            err_x = self.target_goal[0]-self.init_position[0]
            err_y = self.target_goal[1]-self.init_position[1]
            self.target_error = np.sqrt(err_x*err_x + err_y*err_y)

            angle_error = Task.calc_delta_heading(self, self.target_goal)

            self.obs = np.array([self.init_position[0], self.init_position[1], self.init_speed, self.heading_rad, self.target_error, angle_error], dtype=np.float32)

        elif self.task == 'avoid':
            rand_x = int(random.choice(['1', '-1']))*random.randint(5,7)
            rand_y = int(random.choice(['1', '-1']))*random.randint(5,7)
            self.target_goal = [rand_x, rand_y]
            err_x = self.target_goal[0]-self.init_position[0]
            err_y = self.target_goal[1]-self.init_position[1]
            self.target_error = np.sqrt(err_x*err_x + err_y*err_y)
            angle_error = Task.calc_delta_heading(self, self.target_goal)

            self.ob_pos = [err_x/2 + random.randint(-5,5)*0.1, err_y/2 + random.randint(-5,5)*0.1]
            self.ob_radius = 1 + random.randint(0,5)*0.1
            self.ob_error = np.sqrt(self.ob_pos[0]*self.ob_pos[0] + self.ob_pos[1]*self.ob_pos[1]) - self.ob_radius
            self.ob_angle = Task.calc_delta_heading(self, self.ob_pos)

            self.obs = np.array([self.init_position[0], self.init_position[1], self.init_speed, self.heading_rad,\
                                 self.target_error, angle_error, self.ob_error, self.ob_angle], dtype=np.float32)
            self.info['obstacle'] = [self.ob_pos, self.ob_radius, self.ob_angle]

            self.obs_norm = normalize_observation(self.obs, self.obs_low_bound, self.obs_high_bound)

        self.info['state'] = self.state
        self.info['goal'] = self.target_goal
        self.info['init_heading'] = random_heading_deg
        self.info['task'] = self.task
        self.info['obs'] = self.obs

        return self.obs_norm, self.info

    def render(self):
        pass

def normalize_observation(obs, obs_min, obs_max):
    return (obs - obs_min) / (obs_max - obs_min)


class Follow_waypoints(autoRCcarEnv):

    def __init__(self, render_mode: Optional[str] = None):
        self.task = 'waypoint'
        ## Common variable
        self.init_position = [0, 0]
        self.init_speed = 0
        self.init_yaw_rate = 0
        self.heading_rad = 0
        self.state = None
        self.obs = None
        self.info = {}
        self.steps_cnt = None
        self.render_mode = render_mode

        ## 'autoRCcar_waypoint' task
        self.max_step = 5000
        self.target_goal = [0, 0]
        self.target_threshold = 0.2
        self.target_error_before = np.inf
        self.target_angle_before = np.inf

        # Vehicle data
        self.simulation_hz = 100                 # [Hz]
        self.steering_bound = [-deg2rad(45), deg2rad(45)]    # [deg->rad]
        self.speed_bound = [0, 1]
        self.wheelbase = 0.3                     # [m]

        # Action space
        action_low_bound = np.array([self.steering_bound[0], self.speed_bound[0]], dtype=np.float32)
        action_high_bound = np.array([self.steering_bound[1], self.speed_bound[1]], dtype=np.float32)
        self.action_space = spaces.Box(low=action_low_bound, high=action_high_bound)                 # steering_angle, speed

        # Observation space
        self.map_size = 20
        self.obs_low_bound = np.array([-self.map_size, -self.map_size, self.speed_bound[0], 0, -self.map_size, 0], dtype=np.float32)
        self.obs_high_bound = np.array([self.map_size, self.map_size, self.speed_bound[1], 2*np.pi, self.map_size, 2*np.pi], dtype=np.float32)
        self.observation_space = spaces.Box(low=self.obs_low_bound, high=self.obs_high_bound)  # x, y, speed, yaw, distance error, angle error



class avoid_obstacle(autoRCcarEnv):

    def __init__(self, render_mode: Optional[str] = None):
        self.task = 'avoid'
        ## Common variable
        self.init_position = [0, 0]
        self.init_speed = 0
        self.init_yaw_rate = 0
        self.heading_rad = 0
        self.state = None
        self.obs = None
        self.obs_norm = None
        self.info = {}
        self.steps_cnt = None
        self.render_mode = render_mode

        ## 'autoRCcar_avoid' task
        self.max_step = 500
        self.target_goal = [0, 0]
        self.target_threshold = 0.2
        self.target_error_before = np.inf
        self.target_error_before2 = [np.inf, np.inf]
        self.action_before = [np.inf, np.inf]

        self.ob_pos = [0, 0]
        self.ob_radius = 1
        self.ob_error = None

        # Vehicle data
        self.simulation_hz = 10  # [Hz]
        self.steering_bound = [-deg2rad(45), deg2rad(45)] # [deg->rad]
        self.speed_bound = [0, 1]
        self.wheelbase = 0.3      # [m]

        # Action space
        action_low_bound = np.array([self.steering_bound[0], self.speed_bound[0]], dtype=np.float32)
        action_high_bound = np.array([self.steering_bound[1], self.speed_bound[1]], dtype=np.float32)
        self.action_space = spaces.Box(low=action_low_bound, high=action_high_bound)                 # steering_angle, speed

        # Observation space
        # [x, y, speed, yaw, yaw_rate, distance error, angle error, obstacle distance]
        # self.map_size = 20
        # self.obs_low_bound = np.array([-self.map_size, -self.map_size, self.speed_bound[0], 0, -50, -self.map_size*2, 0, -self.map_size*2], dtype=np.float32)
        # self.obs_high_bound = np.array([self.map_size, self.map_size, self.speed_bound[1], 2*np.pi, 50, self.map_size*2, 2*np.pi, self.map_size*2], dtype=np.float32)
        # self.observation_space = spaces.Box(low=self.obs_low_bound, high=self.obs_high_bound)
        
        # # [x, y, speed, yaw, x_error, y_error, angle error, obstacle_distance(w bound)]
        # self.map_size = 8
        # self.obs_low_bound = np.array([-self.map_size*1.5, -self.map_size*1.5, self.speed_bound[0], 0, -self.map_size*2, -self.map_size*2, 0, -self.map_size*2], dtype=np.float32)
        # self.obs_high_bound = np.array([self.map_size*1.5, self.map_size*1.5, self.speed_bound[1], 2*np.pi, self.map_size*2, self.map_size*2, 2*np.pi, self.map_size*2], dtype=np.float32)
        # self.observation_space = spaces.Box(low=self.obs_low_bound, high=self.obs_high_bound)

        # [x, y, speed, yaw, distance error, angle error, obstacle distance(w bound), obstacle angle]
        self.map_size = 20
        self.obs_low_bound = np.array([-self.map_size, -self.map_size, self.speed_bound[0], 0, -self.map_size, 0, -self.map_size, 0], dtype=np.float32)
        self.obs_high_bound = np.array([self.map_size, self.map_size, self.speed_bound[1], 2*np.pi, self.map_size, 2*np.pi, self.map_size, 2*np.pi], dtype=np.float32)
        self.observation_space = spaces.Box(low=self.obs_low_bound, high=self.obs_high_bound)
