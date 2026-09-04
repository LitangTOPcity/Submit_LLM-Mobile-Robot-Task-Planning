"""my_navigation_skills 接口层单元测试（零依赖，无需 pytest，无需 ROS）。

运行方式（任意目录下）：
    python tests/test_my_navigation.py
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from my_navigation_skills.models import Reason  # noqa: E402
from my_navigation_skills.mock_robot import MockRobot  # noqa: E402
from my_navigation_skills.pid_controller import PidController  # noqa: E402
from my_navigation_skills.skill_registry import (  # noqa: E402
    SKILL_REGISTRY, SkillExecutor, compact_registry, json_schemas,
    tool_schemas, validate_arguments,
)

# 快速模拟：10 倍加速
ROBOT = MockRobot(time_scale=10.0)


def _fresh_executor(log_path=None):
    return SkillExecutor(MockRobot(time_scale=10.0), log_path=log_path)


# ---------------- 注册表 ----------------

def test_registry_has_exactly_three_skills():
    assert set(SKILL_REGISTRY) == {"navigate_to", "follow_waypoints", "stop"}


def test_registry_entries_are_complete():
    for name, meta in SKILL_REGISTRY.items():
        for field in ("description", "parameters", "preconditions",
                      "success_condition", "timeout"):
            assert field in meta, f"{name} missing '{field}'"


def test_registry_workspace_limits():
    # 工作区 = 实际房间范围（TurtleBot3 房子世界，odom 系）
    assert SKILL_REGISTRY["navigate_to"]["parameters"]["x"]["max"] == 4.0
    assert SKILL_REGISTRY["navigate_to"]["parameters"]["y"]["max"] == 5.2


def test_registry_tolerances_match_success_condition():
    tol = SKILL_REGISTRY["navigate_to"]["tolerances"]
    assert tol["position_m"] == 0.1 and tol["heading_rad"] == 0.1
    # 数值容差与人类可读的成功条件必须一致（契约一致性）
    cond = SKILL_REGISTRY["navigate_to"]["success_condition"]
    assert "0.1 m" in cond and "0.1 rad" in cond


# ---------------- 校验 ----------------

def test_validation_accepts_valid():
    assert validate_arguments("navigate_to", {"x": 1.0, "y": -0.5, "theta": 0.0}) == []
    assert validate_arguments("follow_waypoints",
                              {"waypoints": [{"x": 0.5, "y": 0.0, "theta": 0.0},
                                             {"x": 1.0, "y": 1.0, "theta": 1.57}]}) == []


def test_validation_rejects_unknown_skill():
    errs = validate_arguments("teleport", {"x": 0.0})
    assert any("unknown skill" in e for e in errs)


def test_validation_rejects_unknown_argument():
    errs = validate_arguments("navigate_to", {"x": 1.0, "y": 1.0, "theta": 0.0, "evil": 1})
    assert any("unknown argument" in e for e in errs)


def test_validation_rejects_missing_required():
    errs = validate_arguments("navigate_to", {"x": 1.0, "y": 1.0})
    assert any("missing required" in e for e in errs)


def test_validation_rejects_wrong_type():
    errs = validate_arguments("navigate_to", {"x": "abc", "y": 1.0, "theta": 0.0})
    assert any("must be numeric" in e for e in errs)


def test_validation_rejects_out_of_workspace():
    errs = validate_arguments("navigate_to", {"x": 10.0, "y": 1.0, "theta": 0.0})
    assert any("maximum" in e for e in errs)


def test_validation_rejects_bad_waypoints():
    errs = validate_arguments("follow_waypoints", {"waypoints": []})
    assert any("at least 1" in e for e in errs)
    # 复现 LLM 真实翻车案例：waypoint 缺 theta（旧格式数组只有 2 个元素）
    errs = validate_arguments("follow_waypoints", {"waypoints": [[0.5, 0.0]]})
    assert any("must be an object" in e for e in errs)
    # 新格式：对象缺字段
    errs = validate_arguments("follow_waypoints",
                              {"waypoints": [{"x": 0.5, "y": 0.0}]})
    assert any("missing required field 'theta'" in e for e in errs)
    # 越界（对象字段的 min/max 检查）
    errs = validate_arguments("follow_waypoints", {"waypoints": [{"x": 5.0, "y": 0.0, "theta": 0.0}]})
    assert any("maximum" in e for e in errs)
    errs = validate_arguments("follow_waypoints", {"waypoints": [{"x": 0.0, "y": 0.0, "theta": 9.0}]})
    assert any("theta" in e for e in errs)


# ---------------- 执行器 ----------------

def test_unknown_skill_result():
    r = _fresh_executor().execute("teleport", {"x": 0.0})
    assert r.success is False and r.reason == Reason.UNKNOWN_SKILL


def test_bad_args_result():
    r = _fresh_executor().execute("navigate_to", {"x": "abc", "y": 1.0, "theta": 0.0})
    assert r.success is False and r.reason == Reason.INVALID_ARGUMENTS


def test_successful_navigate_with_metrics():
    ex = _fresh_executor()
    r = ex.execute("navigate_to", {"x": 1.0, "y": 0.5, "theta": 1.57})
    assert r.success is True and r.reason == Reason.OK
    assert r.details["position_error_m"] <= 0.1
    assert r.details["heading_error_rad"] <= 0.1
    assert r.elapsed_time_s > 0.0


def test_timeout_result_has_metrics():
    r = _fresh_executor().execute("navigate_to", {"x": 3.0, "y": 3.0, "theta": 0.0},
                                  timeout_s=1.0)
    assert r.success is False and r.reason == Reason.TIMEOUT
    assert "position_error_m" in r.details


def test_follow_waypoints_success():
    r = _fresh_executor().execute(
        "follow_waypoints",
        {"waypoints": [{"x": 0.5, "y": 0.0, "theta": 0.0},
                       {"x": 0.5, "y": 0.5, "theta": 0.0}]})
    assert r.success is True and r.reason == Reason.OK
    assert r.details.get("last_reached") == 1
    assert r.details.get("waypoints_visited") == 2


def test_follow_waypoints_failure_reports_last_reached():
    # 第一个点（0.3, 0）轻松到达；第二点（3, 3）太远到不了
    # -> 失败于 waypoint 1，last_reached = 0
    r = _fresh_executor().execute(
        "follow_waypoints",
        {"waypoints": [{"x": 0.3, "y": 0.0, "theta": 0.0},
                       {"x": 3.0, "y": 3.0, "theta": 0.0}]},
        timeout_s=2.0)
    assert r.success is False
    assert r.details.get("last_reached") == 0
    assert r.details.get("failed_at") == 1


def test_stop_success():
    r = _fresh_executor().execute("stop", {})
    assert r.success is True and r.reason == Reason.OK


def test_mutex_rejects_concurrent_skill():
    ex = _fresh_executor()
    t = threading.Thread(target=ex.execute,
                         args=("navigate_to", {"x": 2.0, "y": 0.0, "theta": 0.0}, 3.0))
    t.start()
    time.sleep(0.2)
    r = ex.execute("navigate_to", {"x": 0.0, "y": 0.0, "theta": 0.0})
    t.join()
    assert r.success is False and r.reason == Reason.PRECONDITION_FAILED


def test_result_has_required_fields():
    r = _fresh_executor().execute("stop", {})
    d = r.to_dict()
    assert {"success", "reason", "elapsed_time_s"} <= set(d.keys())


def test_logging_writes_jsonl(tmp_path="logs"):
    log = os.path.join(tmp_path, "test_calls.jsonl")
    if os.path.exists(log):
        os.remove(log)
    ex = SkillExecutor(MockRobot(time_scale=10.0), log_path=log)
    ex.execute("navigate_to", {"x": 0.5, "y": 0.0, "theta": 0.0})
    with open(log, encoding="utf-8") as f:
        line = json.loads(f.readline())
    assert line["skill"] == "navigate_to"
    assert set(line["result"].keys()) >= {"success", "reason", "elapsed_time_s"}


# ---------------- 给 LLM 的格式 ----------------

def test_tool_schemas_format():
    schemas = tool_schemas()
    assert len(schemas) == 3
    nav = next(s for s in schemas if s["function"]["name"] == "navigate_to")
    assert nav["type"] == "function"
    assert nav["function"]["parameters"]["required"] == ["x", "y", "theta"]
    assert nav["function"]["parameters"]["properties"]["x"]["maximum"] == 4.0


def test_json_schemas_format():
    schemas = json_schemas()
    assert set(schemas) == {"navigate_to", "follow_waypoints", "stop"}
    nav = schemas["navigate_to"]
    assert nav["type"] == "object"
    assert nav["required"] == ["x", "y", "theta"]
    assert nav["properties"]["x"]["maximum"] == 4.0
    assert nav["additionalProperties"] is False
    wp = schemas["follow_waypoints"]["properties"]["waypoints"]
    assert wp["type"] == "array" and wp["minItems"] == 1 and wp["maxItems"] == 10


def test_compact_registry_parseable():
    data = json.loads(compact_registry())
    assert len(data) == 3
    assert all({"skill", "description", "parameters", "timeout_s"} <= set(e) for e in data)


# ---------------- PID 控制器（纯逻辑，无 ROS） ----------------

def test_pid_proportional_control():
    pid = PidController(kp=1.0, min_out=0.0, max_out=2.0)
    out = pid.update(0.5, 0.1)
    assert abs(out - 0.5) < 1e-9  # P 项主导


def test_pid_deadband_zero_and_reset():
    pid = PidController(kp=1.0, ki=1.0, deadband=0.02, max_out=10.0, integral_limit=1.0)
    for _ in range(20):
        pid.update(0.5, 0.1)          # 先累积积分
    assert pid._integral > 0.1
    out = pid.update(0.01, 0.1)       # 误差 < 死区
    assert out == 0.0
    assert pid._integral == 0.0       # 死区会清空积分
    # 之后从干净状态重新起步：只有本周期累积的一小步积分
    out2 = pid.update(0.5, 0.1)       # 纯 P = 0.5 + ki*e*dt = 0.05
    assert abs(out2 - 0.55) < 1e-9


def test_pid_integral_grows_then_clamps():
    pid = PidController(kp=0.0, ki=1.0, integral_limit=0.5, max_out=10.0)
    for _ in range(100):          # 长时间累积
        pid.update(1.0, 0.1)
    # 积分被钳位在 integral_limit（抗积分饱和）
    assert pid._integral <= 0.5 + 1e-9


def test_pid_reset_clears_state():
    pid = PidController(kp=1.0, ki=1.0, integral_limit=1.0, max_out=10.0)
    for _ in range(10):
        pid.update(1.0, 0.1)
    assert pid._integral > 0.0
    pid.reset()
    assert pid._integral == 0.0 and pid._has_prev is False


def test_pid_derivative_damps():
    # 误差快速变小 -> 微分项为负 -> 输出小于纯 P
    pid_p = PidController(kp=1.0, kd=0.0, max_out=10.0)
    pid_pd = PidController(kp=1.0, kd=0.5, max_out=10.0)
    pid_p.update(0.5, 0.1); pid_pd.update(0.5, 0.1)
    out_p = pid_p.update(0.4, 0.1)
    out_pd = pid_pd.update(0.4, 0.1)
    assert out_pd < out_p  # D 项在误差减小时提供阻尼


def main() -> None:
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"[PASS] {name}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {name}: {e}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"[ERROR] {name}: {e!r}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
