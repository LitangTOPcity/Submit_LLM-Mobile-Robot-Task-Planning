"""Part 2 —— 离散 PID 控制器（位置式）
========================================

为导航控制提供完整 PID 能力：

  * 角度轴（转向 / 转正）：P + I + D
      - I 项消除"差一点转不到"的稳态角度误差；
      - D 项抑制过冲 / 振荡；
      - 抗积分饱和（积分钳位），防止长距离或卡住时积分爆炸；
      - 微分项限幅，防 odometry 噪声被放大；
      - 死区 + 最小输出（克服轮子静摩擦）。
  * 线速度轴：P + D（距离比例 + 距离变化率阻尼）。不加 I：
      到位前长时间累积距离积分会在到位瞬间造成过冲。

所有可调增益集中在 NavControlConfig —— 换机 / 调参只改这一处（单一配置点）。
"""
from __future__ import annotations


class NavControlConfig:
    """导航控制参数（单位：距离 m，角度 rad，时间 s）。"""

    # ---- 角度轴：输出为角速度 (rad/s) ----
    ANG_KP = 1.0               # 比例增益
    ANG_KI = 0.02              # 积分增益：消除残余角度稳态误差
    ANG_KD = 0.03              # 微分增益：阻尼过冲
    ANG_MIN_OUT = 0.15         # 最小角速度：克服轮子静摩擦/死区
    ANG_MAX_OUT = 1.0          # 最大角速度
    ANG_DEADBAND = 0.02        # 误差死区：到位附近停止输出并清积分
    ANG_INTEGRAL_LIMIT = 0.5   # 抗积分饱和：积分值上下限
    ANG_DERIVATIVE_LIMIT = 1.5 # 微分项限幅 (rad/s)：防噪声放大

    # ---- 线速度轴：输出为线速度 (m/s) ----
    LIN_KP = 0.35              # 距离比例增益：越近越慢
    LIN_KD = 0.08              # 距离变化率阻尼：接近目标时提前减速（防过冲）
    V_MIN = 0.05               # 最小线速度
    V_MAX = 0.25               # 最大线速度


class PidController:
    """位置式离散 PID。

    用法（每个控制周期调用一次）：
        out = pid.update(error, dt)     # error 带符号；dt 为真实时间步长(s)
    换目标前必须调用 pid.reset() 清空积分与微分记忆。
    """

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 min_out: float = 0.0, max_out: float = 1.0,
                 deadband: float = 0.0, integral_limit: float = 0.0,
                 derivative_limit: float | None = None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_out, self.max_out = min_out, max_out
        self.deadband = deadband
        self.integral_limit = integral_limit
        self.derivative_limit = derivative_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def reset(self) -> None:
        """换目标时调用：清空积分与微分记忆，避免跨任务污染。"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def update(self, error: float, dt: float) -> float:
        """输入当前误差与时间步长，输出控制量（已限幅）。"""
        # 死区：误差已足够小 -> 停止输出并清积分（防到位后抖动/饱和）
        if abs(error) < self.deadband:
            self.reset()
            return 0.0
        if dt <= 0.0:
            dt = 0.1  # 防御：非法步长退化为默认值

        # 积分项（带抗积分饱和）
        self._integral += error * dt
        if self.integral_limit > 0.0:
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))

        # 微分项（带限幅，防噪声放大）
        derivative = 0.0
        if self._has_prev and dt > 0.0:
            derivative = (error - self._prev_error) / dt
            if self.derivative_limit is not None:
                derivative = max(-self.derivative_limit,
                                 min(self.derivative_limit, derivative))
        self._prev_error = error
        self._has_prev = True

        out = self.kp * error + self.ki * self._integral + self.kd * derivative

        # 限幅
        out = max(-self.max_out, min(self.max_out, out))
        # 最小输出：指令低于电机死区时无效，给到 min_out（方向随误差）
        if self.min_out > 0.0 and abs(out) < self.min_out:
            out = self.min_out if error >= 0.0 else -self.min_out
        return out
