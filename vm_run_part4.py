#!/usr/bin/env python3
"""VM 运行脚本：在 Webots 仿真里演示 Part 3 + Part 4（走真实 ROS 2 节点，非 Mock）。

注意：机器人是 Webots 中的虚拟 TurtleBot3（仿真），不是实体机器人。

前提（在虚拟机里）：
  1) 终端 A 已启动仿真：ros2 launch webots_ros2_robomaster robot_launch.py
  2) 终端 B 已 source：source /opt/ros/humble/setup.bash
  3) 本文件与 my_navigation_skills/ 在同一目录（或已把 part3_example 整个拷入）

运行：
  python3 vm_run_part4.py            # Mock 模式（无需 API key，可先验证整条链路）
  OPENAI_API_KEY=sk-... python3 vm_run_part4.py   # LLM 模式（真实 function calling）

演示内容（任务书 Part 4 要求的三类请求）：
  1. "Go to storage."                           -> 成功
  2. "Visit workbench, then storage, and stop." -> 成功（含 stop）
  3. "Go to kitchen."                           -> 拒绝（非法位置，列出合法位置）

注意：navigate_to 有 20s 默认超时，三个场景全部跑完约需 1~2 分钟（真实仿真速度）。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402

from my_navigation_skills.llm_client import OpenAIClient  # noqa: E402
from my_navigation_skills.nav_skills import NavigationSkills  # noqa: E402
from my_navigation_skills.planner import TaskPlanner  # noqa: E402
from my_navigation_skills.skill_registry import SkillExecutor  # noqa: E402

SCENARIOS = [
    ("成功：单点", "Go to storage."),
    ("成功：多点并停止", "Visit workbench, then storage, and stop."),
    ("拒绝：非法位置", "Go to kitchen."),
]


def main() -> None:
    rclpy.init()
    skill_node = NavigationSkills()
    executor = SkillExecutor(skill_node, log_path="logs/skill_calls.jsonl")

    # 等 /odom 先来几帧，避免初始位姿还是 (0,0,0)
    print("等待 /odom 数据...")
    for _ in range(100):
        rclpy.spin_once(skill_node, timeout_sec=0.02)
        time.sleep(0.02)

    use_llm = bool(os.environ.get("OPENAI_API_KEY"))
    planner = TaskPlanner(
        executor,
        llm_client=OpenAIClient() if use_llm else None,
        log_path="logs/task_planner.jsonl",
        verbose=True,
    )
    print(f"\n>>> 模式：{'LLM（真实 function calling）' if use_llm else 'Mock（规则解析）'}")

    for title, request in SCENARIOS:
        print(f"\n{'=' * 72}\n>>> {title}: \"{request}\"")
        planner.run(request)

    skill_node.destroy_node()
    rclpy.shutdown()
    print("\n完成。日志：logs/skill_calls.jsonl（技能调用）与 logs/task_planner.jsonl（任务规划）")


if __name__ == "__main__":
    main()
