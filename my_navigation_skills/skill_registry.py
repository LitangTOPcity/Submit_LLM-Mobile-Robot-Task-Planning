#!/usr/bin/env python3
"""
Part 3: Structured Skill Interface —— 技能注册表、参数校验、统一执行器
======================================================================

组成：
  * validate_arguments：收集全部参数错误（类型/范围/工作区/waypoints 结构）；
  * SkillExecutor：allowlist 检查 → 参数校验 → 前置条件 → 互斥 →
    执行 → 规范化结构化结果 → JSONL 日志；
  * tool_schemas() / compact_registry()：供 Part 4 直接喂给 LLM；
  * reason 枚举化（见 models.py），结果统一为 SkillResult。

工作区边界：见下方 WORKSPACE_LIMITS（按真实房间墙体测量，odom 系）。
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Optional

from .models import Reason, SkillResult

# 工作区边界 = 实际房间范围（odom 系，从 TurtleBot3 房子世界测得）
#   西墙 x=-1.36（含门洞）、东墙 x=3.74、南墙 y=-5.1、北墙 y=5.1
WORKSPACE_LIMITS = {"x_min": -2.0, "x_max": 4.0, "y_min": -5.2, "y_max": 5.2}
THETA_MIN, THETA_MAX = -3.14159, 3.14159

# 技能注册表：每个技能的契约（名称是字典键，其余五要素都在这里）
# 参数描述会进入 tool schema，直接指导 LLM 生成合法参数（尽量写清楚）。
SKILL_REGISTRY = {
    "navigate_to": {
        "description": "Move robot to a specific (x, y) pose with target orientation theta",
        "parameters": {
            "x": {"type": "float", "min": WORKSPACE_LIMITS["x_min"], "max": WORKSPACE_LIMITS["x_max"], "required": True,
                  "description": "Target x position in meters (workspace range)"},
            "y": {"type": "float", "min": WORKSPACE_LIMITS["y_min"], "max": WORKSPACE_LIMITS["y_max"], "required": True,
                  "description": "Target y position in meters (workspace range)"},
            "theta": {"type": "float", "min": THETA_MIN, "max": THETA_MAX, "required": True,
                      "description": "Target heading in radians"},
        },
        "preconditions": ["robot initialized", "no other skill executing", "target within workspace"],
        "success_condition": "position error <= 0.1 m and heading error <= 0.1 rad within timeout",
        "tolerances": {"position_m": 0.1, "heading_rad": 0.1},  # 与 nav_skills.py 的容差保持一致
        "timeout": 20.0,
    },
    "follow_waypoints": {
        "description": "Visit a sequence of waypoints in order; reports last waypoint reached on failure",
        "parameters": {
            "waypoints": {
                "type": "list",
                "item_type": "object",  # 每个 waypoint 是 {x, y, theta} 对象（避免 LLM 漏掉 theta）
                "min_len": 1,
                "max_len": 10,
                "required": True,
                "description": "Ordered list of target poses; each item MUST be an object with x, y, theta",
                "item_fields": [
                    {"name": "x", "type": "float", "min": WORKSPACE_LIMITS["x_min"], "max": WORKSPACE_LIMITS["x_max"], "required": True,
                     "description": "x position in meters"},
                    {"name": "y", "type": "float", "min": WORKSPACE_LIMITS["y_min"], "max": WORKSPACE_LIMITS["y_max"], "required": True,
                     "description": "y position in meters"},
                    {"name": "theta", "type": "float", "min": THETA_MIN, "max": THETA_MAX, "required": True,
                     "description": "heading in radians"},
                ],
            },
        },
        "preconditions": ["at least one waypoint", "all waypoints within workspace", "no other skill executing"],
        "success_condition": "all waypoints reached within timeout",
        "timeout": 60.0,  # 总超时；执行器按 waypoint 数均分到每个点
    },
    "stop": {
        "description": "Immediately stop the robot",
        "parameters": {},
        "preconditions": [],
        "success_condition": "robot velocity zero",
        "timeout": 1.0,
    },
}


# ============================ 参数校验 ============================

def validate_arguments(skill_name: str, args: dict) -> list[str]:
    """按技能定义校验参数，返回错误列表（空列表 = 通过）。

    覆盖：未知技能、未知参数、缺少必需参数、类型错误、范围越界、
    waypoints 结构与每个点的工作区检查。
    """
    if skill_name not in SKILL_REGISTRY:
        return [f"unknown skill '{skill_name}'; allowed: {list(SKILL_REGISTRY)}"]

    errors: list[str] = []
    param_defs = SKILL_REGISTRY[skill_name].get("parameters", {})

    # 未知参数（LLM 幻觉的典型产物）
    for key in args:
        if key not in param_defs:
            errors.append(f"unknown argument '{key}'")

    for p_name, p_info in param_defs.items():
        if p_info.get("required", False) and p_name not in args:
            errors.append(f"missing required parameter '{p_name}'")
            continue
        if p_name not in args:
            continue
        value = args[p_name]

        if p_info["type"] == "float":
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append(f"parameter '{p_name}' must be numeric, got {type(value).__name__}")
                continue
            if "min" in p_info and value < p_info["min"]:
                errors.append(f"parameter '{p_name}' {value} < minimum {p_info['min']}")
            if "max" in p_info and value > p_info["max"]:
                errors.append(f"parameter '{p_name}' {value} > maximum {p_info['max']}")

        elif p_info["type"] == "list":
            if not isinstance(value, list):
                errors.append(f"parameter '{p_name}' must be a list, got {type(value).__name__}")
                continue
            if "min_len" in p_info and len(value) < p_info["min_len"]:
                errors.append(f"parameter '{p_name}' needs at least {p_info['min_len']} waypoints")
            if "max_len" in p_info and len(value) > p_info["max_len"]:
                errors.append(f"parameter '{p_name}' allows at most {p_info['max_len']} waypoints")

            item_type = p_info.get("item_type", "list")
            item_fields = p_info.get("item_fields", [])
            for i, wp in enumerate(value):
                if item_type == "object":
                    # 每个 waypoint 必须是 {x, y, theta} 对象
                    if not isinstance(wp, dict):
                        errors.append(f"waypoint[{i}] must be an object {{x, y, theta}}")
                        continue
                    for f in item_fields:
                        if f.get("required") and f["name"] not in wp:
                            errors.append(f"waypoint[{i}] missing required field '{f['name']}'")
                            continue
                        if f["name"] not in wp:
                            continue
                        v = wp[f["name"]]
                        if isinstance(v, bool) or not isinstance(v, (int, float)):
                            errors.append(f"waypoint[{i}].{f['name']} must be numeric")
                            continue
                        if "min" in f and v < f["min"]:
                            errors.append(f"waypoint[{i}].{f['name']}={v} < minimum {f['min']}")
                        if "max" in f and v > f["max"]:
                            errors.append(f"waypoint[{i}].{f['name']}={v} > maximum {f['max']}")
                else:
                    # 旧格式兼容：列表形式的 [x, y, theta]
                    if not isinstance(wp, (list, tuple)) or len(wp) != 3:
                        errors.append(f"waypoint[{i}] must be [x, y, theta]")
                        continue
                    x, y, th = wp
                    if not (WORKSPACE_LIMITS["x_min"] <= x <= WORKSPACE_LIMITS["x_max"]):
                        errors.append(f"waypoint[{i}] x={x} outside workspace x range")
                    if not (WORKSPACE_LIMITS["y_min"] <= y <= WORKSPACE_LIMITS["y_max"]):
                        errors.append(f"waypoint[{i}] y={y} outside workspace y range")
                    if not (THETA_MIN <= th <= THETA_MAX):
                        errors.append(f"waypoint[{i}] theta={th} outside [-pi, pi]")

    return errors


# ============================ 机器可读的三种格式 ============================

def _param_json_schema(p_name: str, p: dict) -> dict:
    """把注册表里的一个参数定义转成 JSON Schema（float / list 两种）。

    list 且 item_type == "object" 时，items 会展开成 {x, y, theta} 对象——
    这样 LLM 才能正确生成每个 waypoint 的三个字段（不会再漏 theta）。
    """
    desc = p.get("description", f"parameter {p_name}")
    if p["type"] == "float":
        schema: dict[str, Any] = {"type": "number", "description": desc}
        if "min" in p:
            schema["minimum"] = p["min"]
        if "max" in p:
            schema["maximum"] = p["max"]
    else:  # list
        schema: dict[str, Any] = {"type": "array", "description": desc}
        if p.get("item_type") == "object":
            item_props, item_required = {}, []
            for f in p.get("item_fields", []):
                f_schema: dict[str, Any] = {"type": "number",
                                            "description": f.get("description", f["name"])}
                if "min" in f:
                    f_schema["minimum"] = f["min"]
                if "max" in f:
                    f_schema["maximum"] = f["max"]
                item_props[f["name"]] = f_schema
                if f.get("required"):
                    item_required.append(f["name"])
            schema["items"] = {"type": "object", "properties": item_props,
                               "required": item_required,
                               "additionalProperties": False}
        else:
            schema["items"] = {"type": "array", "items": {"type": "number"},
                               "minItems": 3, "description": "[x, y, theta]"}
        if "min_len" in p:
            schema["minItems"] = p["min_len"]
        if "max_len" in p:
            schema["maxItems"] = p["max_len"]
    return schema


def compact_registry() -> str:
    """紧凑技能注册表（任务书要求）：放进系统提示词，让 LLM 知道有哪些技能。"""
    out = [
        {
            "skill": name,
            "description": meta["description"],
            "parameters": meta["parameters"],
            "timeout_s": meta["timeout"],
        }
        for name, meta in SKILL_REGISTRY.items()
    ]
    return json.dumps(out, ensure_ascii=False, indent=2)


def tool_schemas() -> list[dict]:
    """OpenAI function-calling 格式：Part 4 里直接作为 tools=[...] 传给 LLM。"""
    schemas = []
    for name, meta in SKILL_REGISTRY.items():
        properties, required = {}, []
        for p_name, p in meta["parameters"].items():
            properties[p_name] = _param_json_schema(p_name, p)
            if p.get("required"):
                required.append(p_name)
        schemas.append({
            "type": "function",
            "function": {
                "name": name,
                "description": meta["description"],
                "parameters": {"type": "object", "properties": properties, "required": required},
            },
        })
    return schemas


def json_schemas() -> dict[str, dict]:
    """每个技能的标准 JSON Schema（任务书提到的"machine-readable interface"）。

    与 tool_schemas() 同源生成，可配合 jsonschema 库做严格校验，
    也可以直接作为接口文档交给其他模块/团队使用。
    """
    schemas: dict[str, dict] = {}
    for name, meta in SKILL_REGISTRY.items():
        properties, required = {}, []
        for p_name, p in meta["parameters"].items():
            properties[p_name] = _param_json_schema(p_name, p)
            if p.get("required"):
                required.append(p_name)
        schemas[name] = {
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "title": name,
            "description": meta["description"],
            "type": "object",
            "properties": properties,
            "required": required,
            "additionalProperties": False,
        }
    return schemas


# ============================ 统一执行器 ============================

# node 实现返回的 reason 字符串 -> Reason 枚举
_REASON_MAP = {
    "arrived": Reason.OK,
    "stopped": Reason.OK,
    "timeout": Reason.TIMEOUT,
    "interrupted": Reason.INTERRUPTED,
}


class SkillExecutor:
    """LLM 层触达机器人的唯一通道：校验 → 前置条件 → 互斥 → 执行 → 结果 → 日志。

    node: 任意实现了 navigate_to / follow_waypoints / stop 三个方法的对象
          （真实场景传 NavigationSkills 节点；离线测试传 MockRobot）。
    """

    def __init__(self, node: Any, log_path: str = "logs/skill_calls.jsonl", verbose: bool = False):
        self.node = node
        self.log_path = log_path
        self.verbose = verbose
        self._busy = False
        self._busy_skill: Optional[str] = None

    # ---------- 对外唯一入口 ----------
    def execute(self, skill_name: str, args: dict, timeout_s: Optional[float] = None) -> SkillResult:
        """执行一个技能调用。timeout_s 可覆盖注册表默认超时（重试/演示用）。"""
        start = time.monotonic()
        result = self._run(skill_name, args, start, timeout_s)
        self._log(skill_name, args, result)
        return result

    # ---------- 内部流水线 ----------
    def _run(self, skill_name: str, args: dict, start: float,
             timeout_s: Optional[float]) -> SkillResult:
        # ① allowlist（注册表本身就是严格白名单）
        if skill_name not in SKILL_REGISTRY:
            return self._fail(skill_name, Reason.UNKNOWN_SKILL,
                              f"unknown skill '{skill_name}'; allowed: {list(SKILL_REGISTRY)}", start)

        # ② 参数校验（类型 + 范围 + 工作区）
        errors = validate_arguments(skill_name, args)
        if errors:
            return self._fail(skill_name, Reason.INVALID_ARGUMENTS, "; ".join(errors), start)

        # ③ 前置条件（代码里真检查，不只是文档）
        pre: list[str] = []
        if self.node is None:
            pre.append("robot not initialized")
        if self._busy and skill_name != "stop":  # stop 例外：允许打断
            pre.append(f"robot busy with '{self._busy_skill}'")
        if pre:
            return self._fail(skill_name, Reason.PRECONDITION_FAILED, "; ".join(pre), start)

        # ④ 执行（dispatch 到 node，超时由执行器统一管理）
        effective_timeout = timeout_s if timeout_s is not None else SKILL_REGISTRY[skill_name]["timeout"]
        self._busy, self._busy_skill = True, skill_name
        try:
            raw = self._dispatch(skill_name, args, effective_timeout)
        except Exception as exc:  # 实现内部错误不允许向上传播
            return self._fail(skill_name, Reason.INTERNAL_ERROR, f"internal error: {exc}", start)
        finally:
            self._busy, self._busy_skill = False, None

        # ⑤ 规范化：node 的 dict -> SkillResult（枚举 reason + 指标进 details）
        return self._normalize(skill_name, raw, start)

    def _dispatch(self, skill_name: str, args: dict, timeout: float) -> dict:
        if skill_name == "navigate_to":
            return self.node.navigate_to(args["x"], args["y"], args["theta"], timeout=timeout)
        if skill_name == "follow_waypoints":
            # waypoints 是 [{x, y, theta}, ...]，转成节点需要的 (x, y, theta) 元组
            waypoints = [(wp["x"], wp["y"], wp["theta"]) for wp in args["waypoints"]]
            per_target = timeout / max(len(waypoints), 1)  # 总超时均分到每个点
            return self.node.follow_waypoints(waypoints, timeout_per_target=per_target)
        if skill_name == "stop":
            self.node.stop()
            return {"success": True, "reason": "stopped", "elapsed_time": 0.0}
        raise ValueError(f"unreachable: {skill_name}")  # 前面已被 allowlist 拦下

    def _normalize(self, skill_name: str, raw: dict, start: float) -> SkillResult:
        success = bool(raw.get("success", False))
        reason = _REASON_MAP.get(raw.get("reason", ""), Reason.INTERNAL_ERROR)
        details = {k: v for k, v in raw.items() if k not in ("success", "reason", "elapsed_time")}
        elapsed = raw.get("elapsed_time")
        if elapsed is None:
            elapsed = time.monotonic() - start
        return SkillResult(skill=skill_name, success=success, reason=reason,
                           elapsed_time_s=round(float(elapsed), 3), details=details)

    def _fail(self, skill_name: str, reason: Reason, message: str, start: float) -> SkillResult:
        return SkillResult(skill=skill_name, success=False, reason=reason,
                           elapsed_time_s=round(time.monotonic() - start, 3),
                           details={"message": message})

    # ---------- 结构化日志（JSONL，每行一次调用） ----------
    def _log(self, skill_name: str, args: dict, result: SkillResult) -> None:
        line = json.dumps({"skill": skill_name, "arguments": args,
                           "result": result.to_dict()}, ensure_ascii=False)
        if self.verbose:
            print(line)
        if self.log_path:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")


# ============================ 冒烟测试（需 ROS 环境） ============================

def main() -> None:
    import rclpy
    from .nav_skills import NavigationSkills

    rclpy.init()
    node = NavigationSkills()
    executor = SkillExecutor(node, verbose=True)

    print(executor.execute("navigate_to", {"x": 0.5, "y": 0.0, "theta": 0.0}).to_json())
    print(executor.execute("follow_waypoints",
                           {"waypoints": [{"x": 0.5, "y": 0.0, "theta": 0.0},
                                          {"x": 0.5, "y": 0.5, "theta": 0.0},
                                          {"x": 0.0, "y": 0.5, "theta": 0.0}]}).to_json())
    print(executor.execute("stop", {}).to_json())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
