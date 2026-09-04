"""离线机器人模拟 —— 与 NavigationSkills 公开接口一致，无 ROS 依赖
=================================================================

用途：
  * 在没启动 Webots / ROS 的机器上验证 Part 3 接口层（注册表/校验/执行器/日志）；
  * 控制逻辑与 nav_skills.py 一致（先转向目标点 → 前进 → 转正朝向）。

time_scale 用于加速模拟（time_scale=10 表示模拟 10 秒只需真实 1 秒），
方便测试超时等慢场景。
"""
from __future__ import annotations

import math
import time


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def turn_command(error_rad: float, gain: float = 1.0,
                 min_speed: float = 0.15, max_speed: float = 1.0,
                 deadband: float = 0.02) -> float:
    """与 nav_skills.py 相同的转向指令（最小角速度 + 死区）。"""
    if abs(error_rad) < deadband:
        return 0.0
    cmd = clamp(gain * error_rad, -max_speed, max_speed)
    if abs(cmd) < min_speed:
        cmd = math.copysign(min_speed, error_rad)
    return cmd


class MockRobot:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0,
                 max_linear: float = 0.2, max_angular: float = 0.5,
                 dt: float = 0.1, time_scale: float = 1.0,
                 pos_tol: float = 0.1, ang_tol: float = 0.1):
        self.x, self.y, self.theta = x, y, theta
        self.max_linear, self.max_angular = max_linear, max_angular
        self.dt, self.time_scale = dt, time_scale
        self.pos_tol, self.ang_tol = pos_tol, ang_tol
        self.v, self.w = 0.0, 0.0

    # ---------------- 与 NavigationSkills 相同的三个公开方法 ----------------

    def navigate_to(self, target_x, target_y, target_theta, timeout=20.0):
        """与 nav_skills.py 相同的两阶段状态机（drive -> rotate，带迟滞）。"""
        start = time.monotonic()
        phase = "drive"
        reenter_dist = self.pos_tol * 2.5

        while True:
            dx, dy = target_x - self.x, target_y - self.y
            dist = math.hypot(dx, dy)
            heading_error = normalize_angle(target_theta - self.theta)

            # 状态切换（带迟滞）
            if dist <= self.pos_tol:
                phase = "rotate"
            elif dist > reenter_dist:
                phase = "drive"

            if phase == "rotate" and abs(heading_error) <= self.ang_tol:
                self.stop()
                return {'success': True, 'reason': 'arrived',
                        'elapsed_time': round(time.monotonic() - start, 3),
                        'position_error_m': round(dist, 3),
                        'heading_error_rad': round(abs(heading_error), 3)}

            if time.monotonic() - start > timeout:
                self.stop()
                return {'success': False, 'reason': 'timeout',
                        'elapsed_time': round(time.monotonic() - start, 3),
                        'position_error_m': round(dist, 3),
                        'heading_error_rad': round(abs(heading_error), 3)}

            if phase == "drive":
                target_direction = math.atan2(dy, dx)
                direction_error = normalize_angle(target_direction - self.theta)
                if abs(direction_error) > 0.1:
                    self._step(0.0, turn_command(direction_error))
                else:
                    # 与 nav_skills 对齐：前进时边走边小修正（平滑转向，避免顿挫）
                    self._step(clamp(0.35 * dist, 0.05, self.max_linear),
                               clamp(1.5 * direction_error, -0.35, 0.35))
            else:  # rotate：锁定在原地转正
                self._step(0.0, turn_command(heading_error))
            time.sleep(self.dt / self.time_scale)

    def follow_waypoints(self, waypoints, timeout_per_target=20.0):
        last_reached = -1
        total_time = 0.0
        for i, (x, y, th) in enumerate(waypoints):
            result = self.navigate_to(x, y, th, timeout_per_target)
            total_time += result.get('elapsed_time', 0.0)
            if not result['success']:
                return {'success': False, 'reason': result['reason'],
                        'last_reached': last_reached, 'failed_at': i,
                        'elapsed_time': round(total_time, 3),
                        'position_error_m': result.get('position_error_m'),
                        'heading_error_rad': result.get('heading_error_rad')}
            last_reached = i
        return {'success': True, 'reason': 'arrived', 'last_reached': last_reached,
                'waypoints_visited': len(waypoints), 'elapsed_time': round(total_time, 3)}

    def stop(self):
        self.v, self.w = 0.0, 0.0

    # ---------------- 内部运动学 ----------------

    def _step(self, v: float, w: float):
        self.v, self.w = v, w
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta = normalize_angle(self.theta + w * self.dt)
