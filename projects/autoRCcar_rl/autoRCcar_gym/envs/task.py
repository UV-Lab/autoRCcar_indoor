import numpy as np
import math
class Task:
    # def init_task(self):

    def _is_goal_distance(self):
        current = self.state[0:2]

        err_x = self.target_goal[0] - current[0]
        err_y = self.target_goal[1] - current[1]
        goal_error = np.sqrt(err_x*err_x + err_y*err_y)

        if np.abs(goal_error) < self.target_threshold:
            return True, goal_error
        else:
            return False, goal_error


    def _is_terminated_distance(self):
        # 직선방정식에서 얼마나 떨어져 있는지
        x = self.state[0]
        y = self.state[1]

        a = np.tan(self.target_direction)
        b = -1
        c = 0

        tmp1 = np.linalg.norm(a*x + b*y + c)
        tmp2 = np.sqrt(a*a + b*b)
        distance_error = tmp1 / tmp2

        if np.abs(distance_error) >= self.target_threshold:
            return True, distance_error
        else:
            return False, distance_error


    def _is_terminated_step(steps):
        if steps == 0:
            return True
        else:
            return False

    def _is_terminated_wp(self, margin):
        flag = False
        world = self.map_size + margin

        if abs(self.state[0]) > world or abs(self.state[1]) > world:
            flag = True

        if self.step_cnt >= self.max_step-1:
            flag = True

        return flag

    def _is_facing_target(self, threshold):
        current = self.state[0:2]
        current_yaw = self.state[3]

        # 현재 위치와 목표점 사이의 방향 벡터 계산
        dx = self.target[0] - current[0]
        dy = self.target[1] - current[1]
        target_heading = math.atan2(dy, dx)
        
        # 현재 방향과 목표 방향 사이의 각도 계산
        yaw_diff = current_yaw - target_heading
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi  # -π ~ π 범위로 조정
        
        # 각도가 일치하는지 확인하여 목표점을 향하고 있는지 여부 반환
        tolerance = math.radians(threshold)  # 허용되는 각도 오차
        if abs(yaw_diff) < tolerance:
            return True, yaw_diff
        else:
            return False, yaw_diff

    def calc_delta_heading(self):
        yaw = self.state[3]
        
        target_x = self.target_goal[0] - self.state[0]
        target_y = self.target_goal[1] - self.state[1]

        angle_rad = math.atan2(target_y, target_x)
        # angle_deg = math.degrees(angle_rad)

        dHd = np.abs(yaw - angle_rad)
        if dHd >= np.pi:
            dHd = 2*np.pi - dHd
        elif dHd <= -np.pi:
            dHd = 2*np.pi + dHd

        return dHd