"""Part 4 —— 命名位置 → 位姿映射
=================================

这些坐标是**按真实仿真世界（Webots TurtleBot3 房子世界）的布局**选的，
坐标系为 odom（原点 = 机器人出生点）。
坐标经程序化净空验证：
  1. 每个位置到障碍物（墙/家具/盆栽）净空 >= 0.5 m；
  2. 任意两个位置之间的直线路径净空 >= 0.35 m（机器人半径约 0.14 m + 余量）。

房间布局要点（odom 系）：西墙 x=-1.36（y∈[-2,-1] 是门洞=entrance），
东墙 x=3.74，南墙 y=-5.1，北墙 y=5.1；中央偏东有桌/沙发/扶手椅家具堆。
"""
from __future__ import annotations

LOCATIONS: dict[str, dict[str, float]] = {
    "entrance": {"x": -0.7, "y": -1.6, "theta": 3.14159},   # 门洞内侧，面向门（西）
    "storage": {"x": 0.0, "y": -3.2, "theta": 0.0},         # 南侧开阔区
    "workbench": {"x": 1.5, "y": -2.8, "theta": 1.57},      # 东南开阔区，带 90° 朝向
    "charging_station": {"x": 2.2, "y": -3.5, "theta": 0.0},  # 东南角（远离家具堆）
}


def location_names() -> list[str]:
    return list(LOCATIONS)


def location_pose(name: str) -> dict[str, float]:
    """按名字返回位姿；未知名字抛 KeyError（调用方应先查 location_names）。"""
    return dict(LOCATIONS[name])


def locations_for_prompt() -> str:
    """给 LLM 系统提示词用的位置清单（紧凑、可直接读）。"""
    lines = []
    for name, pose in LOCATIONS.items():
        lines.append(f"  - {name}: x={pose['x']}, y={pose['y']}, theta={pose['theta']}")
    return "\n".join(lines)
