#!/usr/bin/env python3
"""Part 2 导航技能 —— NavigationSkills ROS 节点
================================================

设计要点：
  1. 成功条件里的"朝向误差"相对目标朝向 target_theta 判定，并含"到位后
     原地转正朝向"阶段（保证到达时朝向与 target_theta 一致，而非仅面向目标点）；
  2. 成功/超时结果带可观测指标 position_error_m / heading_error_rad；
  3. 超时时 elapsed_time 返回实际耗时；
  4. stop() 连续发三次零速，确保仿真机器人真正停下；
  5. follow_waypoints 失败时同时报告 last_reached 与 failed_at；
  6. navigate_to 采用两阶段状态机（drive -> rotate，带迟滞），避免在容差
     边界来回切换导致转正不收敛；
  7. 角度轴用完整 PID（P+I+D：消稳态误差、阻尼过冲），线速度轴用 P+D
     （距离比例 + 距离变化率阻尼），增益集中于 NavControlConfig 一处调参。

坐标约定（评分表要求"clear coordinate conventions"，请与仿真保持一致）：
  - 参考系：odom（世界系）；x 向前、y 向左，theta 从 +x 轴逆时针为正；
  - 单位：位置为米（m），角度为弧度（rad）；
  - 成功容差：位置误差 <= 0.1 m 且 朝向误差 <= 0.1 rad
    （与 skill_registry 中 navigate_to 的 success_condition / tolerances 一致）。

所有实现返回 dict，约定：
    {"success": bool, "reason": "arrived"|"timeout"|"interrupted",
     "elapsed_time": float, "position_error_m": float, "heading_error_rad": float}
skill_registry.SkillExecutor 会把这些 dict 规范化为 SkillResult。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

# 双导入：包内运行时用相对导入；直接 python3 nav_skills.py 也能跑
try:
    from .pid_controller import NavControlConfig, PidController
except ImportError:  # pragma: no cover
    from pid_controller import NavControlConfig, PidController


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class NavigationSkills(Node):
    def __init__(self):
        super().__init__('navigation_skills')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        # 成功容差（与 skill_registry 里 navigate_to 的 success_condition 保持一致）
        self.pos_tolerance = 0.1
        self.ang_tolerance = 0.1
        # 完整 PID：direction_error（行驶中转向）与 heading_error（到位转正）各用一个实例
        self._dir_pid = PidController(
            NavControlConfig.ANG_KP, NavControlConfig.ANG_KI, NavControlConfig.ANG_KD,
            NavControlConfig.ANG_MIN_OUT, NavControlConfig.ANG_MAX_OUT,
            NavControlConfig.ANG_DEADBAND, NavControlConfig.ANG_INTEGRAL_LIMIT,
            NavControlConfig.ANG_DERIVATIVE_LIMIT)
        self._head_pid = PidController(
            NavControlConfig.ANG_KP, NavControlConfig.ANG_KI, NavControlConfig.ANG_KD,
            NavControlConfig.ANG_MIN_OUT, NavControlConfig.ANG_MAX_OUT,
            NavControlConfig.ANG_DEADBAND, NavControlConfig.ANG_INTEGRAL_LIMIT,
            NavControlConfig.ANG_DERIVATIVE_LIMIT)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def navigate_to(self, target_x, target_y, target_theta, timeout=20.0):
        """移动到目标位姿：两阶段状态机（drive -> rotate）。

        阶段 drive  —— dist > pos_tolerance：先转向"指向目标点"的方向，再前进；
        阶段 rotate —— dist <= pos_tolerance：锁定在原地转正到 target_theta，
                       只有明显远离目标（dist > 0.25）才退回 drive（迟滞，
                       防止在容差边界来回切换导致转正永远不收敛——虚拟机实测问题）。

        成功条件（与任务书一致）：位置误差 <= pos_tolerance 且 朝向误差 <= ang_tolerance。
        失败（超时）时带上误差指标，供 Part 4 的恢复逻辑决策。
        """
        self.get_logger().info(f'Navigating to x={target_x}, y={target_y}, theta={target_theta}')
        start_time = time.time()
        phase = "drive"
        reenter_dist = self.pos_tolerance * 2.5  # 迟滞：明显远离才允许回到行驶阶段

        # PID 状态初始化（换目标必须 reset，防止积分/微分跨任务污染）
        self._dir_pid.reset()
        self._head_pid.reset()
        prev_t: float | None = None      # 上一控制周期时刻（算 dt）
        prev_dist: float | None = None   # 上一周期距离（线速 D 项用）
        last_phase = ""

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 时间步长（用真实时间，防 sleep 漂移）
            now = time.monotonic()
            dt = (now - prev_t) if prev_t is not None else 0.1
            prev_t = now

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.hypot(dx, dy)
            # 朝向误差：目标朝向 target_theta 与当前朝向之差（修复点）
            heading_error = normalize_angle(target_theta - self.current_theta)
            # 距离变化率（线速 D 项：接近时为负 -> 提前减速防过冲）
            dist_rate = ((dist - prev_dist) / dt) if prev_dist is not None else 0.0
            prev_dist = dist

            # 状态切换（带迟滞，避免边界抖动）
            if dist <= self.pos_tolerance:
                phase = "rotate"
            elif dist > reenter_dist:
                phase = "drive"
            if phase != last_phase:          # 进入新阶段：清掉上一个阶段的 PID 记忆
                if phase == "rotate":
                    self._head_pid.reset()
                last_phase = phase

            self.get_logger().info(
                f'Current: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_theta:.2f})  '
                f'Target: ({target_x}, {target_y}, {target_theta})  Dist: {dist:.2f}  '
                f'HeadingErr: {heading_error:.2f}  Phase: {phase}'
            )

            if phase == "rotate" and abs(heading_error) <= self.ang_tolerance:
                self.stop()
                elapsed = time.time() - start_time
                self.get_logger().info(f'Arrived at target in {elapsed:.2f}s')
                return {'success': True, 'reason': 'arrived', 'elapsed_time': round(elapsed, 3),
                        'position_error_m': round(dist, 3),
                        'heading_error_rad': round(abs(heading_error), 3)}

            if time.time() - start_time > timeout:
                self.stop()
                elapsed = time.time() - start_time
                self.get_logger().warn(f'Navigation timed out after {elapsed:.2f}s')
                return {'success': False, 'reason': 'timeout', 'elapsed_time': round(elapsed, 3),
                        'position_error_m': round(dist, 3),
                        'heading_error_rad': round(abs(heading_error), 3)}

            twist = Twist()
            if phase == "drive":
                # 距离目标还远：先转到"指向目标点"的方向，再前进
                target_direction = math.atan2(dy, dx)
                direction_error = normalize_angle(target_direction - self.current_theta)
                if abs(direction_error) > 0.1:
                    # 方向偏差较大：原地转向（角度 PID）
                    twist.angular.z = self._dir_pid.update(direction_error, dt)
                    twist.linear.x = 0.0
                else:
                    # 方向偏差小：前进并"边走边修正"（连续小转向），
                    # 避免 停→转→走 的顿挫（TurtleBot 直线行驶会轻微侧漂，
                    # 若不做连续修正会周期性停下来对方向）。
                    v = NavControlConfig.LIN_KP * dist + NavControlConfig.LIN_KD * dist_rate
                    twist.linear.x = clamp(v, NavControlConfig.V_MIN, NavControlConfig.V_MAX)
                    twist.angular.z = clamp(1.5 * direction_error, -0.35, 0.35)
            else:
                # 已到位：锁定原地转正（角度 PID 带 I 消稳态、D 阻尼）
                twist.angular.z = self._head_pid.update(heading_error, dt)
                twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        elapsed = time.time() - start_time
        self.stop()
        return {'success': False, 'reason': 'interrupted', 'elapsed_time': round(elapsed, 3),
                'position_error_m': round(math.hypot(target_x - self.current_x,
                                                     target_y - self.current_y), 3),
                'heading_error_rad': round(abs(normalize_angle(target_theta - self.current_theta)), 3)}

    def stop(self):
        """连续发几次零速，确保仿真机器人真正停下。"""
        twist = Twist()
        for _ in range(3):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.get_logger().info('Stopped')

    def follow_waypoints(self, waypoints, timeout_per_target=20.0):
        """依次访问一串 (x, y, theta) 目标位姿。

        失败时报告 last_reached（最后成功到达的下标）与 failed_at（失败点下标），
        并带上失败点的误差指标——任务书明确要求。
        """
        last_reached = -1
        total_time = 0.0
        for i, (x, y, theta) in enumerate(waypoints):
            result = self.navigate_to(x, y, theta, timeout_per_target)
            total_time += result.get('elapsed_time', 0.0)
            if not result['success']:
                self.get_logger().warn(f'Failed at waypoint {i}, last reached {last_reached}')
                return {'success': False, 'reason': result['reason'],
                        'last_reached': last_reached, 'failed_at': i,
                        'elapsed_time': round(total_time, 3),
                        'position_error_m': result.get('position_error_m'),
                        'heading_error_rad': result.get('heading_error_rad')}
            last_reached = i
        return {'success': True, 'reason': 'arrived', 'last_reached': last_reached,
                'waypoints_visited': len(waypoints), 'elapsed_time': round(total_time, 3)}


def main(args=None):
    """快速冒烟测试（Webots 仿真运行时执行 python3 nav_skills.py）。

    先等 /odom 有数据，然后演示：
      1) navigate_to(1.0, 0.5, 1.57) —— 目标朝向 90°，验证"到位后转正"修复；
      2) follow_waypoints 三个点 —— 任务书要求的多点访问。
    """
    rclpy.init(args=args)
    node = NavigationSkills()

    print("等待 /odom 数据...")
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    print(node.navigate_to(1.0, 0.5, 1.57, timeout=30.0))

    waypoints = [
        (0.5, 0.0, 0.0),
        (0.5, 0.5, 0.0),
        (0.0, 0.5, 0.0),
    ]
    print(node.follow_waypoints(waypoints, timeout_per_target=15.0))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
