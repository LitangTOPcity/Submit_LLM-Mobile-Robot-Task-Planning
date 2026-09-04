"""Part 4 规划器单元测试（零依赖，无需 ROS / LLM key）。

运行方式（任意目录）：
    python tests/test_planner.py
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from my_navigation_skills.mock_robot import MockRobot  # noqa: E402
from my_navigation_skills.planner import TaskPlanner  # noqa: E402
from my_navigation_skills.skill_registry import SkillExecutor  # noqa: E402


def make_planner(max_steps=6, max_retries=1, llm_client=None, log_path=None):
    robot = MockRobot(time_scale=10.0)
    executor = SkillExecutor(robot, log_path=None)
    return TaskPlanner(executor, llm_client=llm_client, max_steps=max_steps,
                       max_retries=max_retries, log_path=log_path), robot


class StubLLM:
    """脚本化 LLM：按顺序返回预设响应，用于测试 LLM 模式的各种分支。"""

    def __init__(self, script):
        self.script = list(script)
        self.calls = 0

    def chat(self, messages, tools=None):
        step = self.script[min(self.calls, len(self.script) - 1)]
        self.calls += 1
        return step


def tool_call(name, arguments, cid="call_1"):
    """模拟 LLM 返回的响应：包含一个 tool call。"""
    return {"content": None,
            "tool_calls": [{"id": cid, "name": name, "arguments": arguments}]}


def final(content="done"):
    return {"content": content, "tool_calls": None}


# ---------------- Mock 模式：任务书三类请求 ----------------

def test_mock_single_location():
    planner, _ = make_planner()
    o = planner.run("Go to storage.")
    assert o.final_status == "success"
    assert o.original_plan == [{"skill": "navigate_to",
                                "arguments": {"x": 0.0, "y": -3.2, "theta": 0.0}}]
    assert o.steps[0].result.success


def test_mock_multi_location_with_stop():
    planner, _ = make_planner()
    o = planner.run("Visit workbench, then storage, and stop.")
    assert o.final_status == "success"
    skills = [s["skill"] for s in o.original_plan]
    assert skills == ["navigate_to", "navigate_to", "stop"]
    assert o.steps[-1].result.success


def test_mock_invalid_location():
    planner, _ = make_planner()
    o = planner.run("Go to kitchen.")
    assert o.final_status == "rejected"
    assert "kitchen" in o.final_message
    assert o.steps == []  # 没执行任何技能


# ---------------- Mock 模式：安全与恢复 ----------------

def test_mock_timeout_retry_once_then_fail():
    # charging_station 距离 (0,0) 约 4.2m，0.8s（真实时间）内到不了 -> 超时 -> 重试 -> 再超时
    planner, _ = make_planner()
    o = planner.run("Go to charging_station.", default_timeout_s=0.8)
    assert o.final_status == "failed"
    assert "stop_and_report" in (o.steps[0].recovery or "")
    assert o.steps[0].attempts == 2  # 重试了一次


def test_mock_plan_exceeds_max_steps():
    planner, _ = make_planner(max_steps=2)
    o = planner.run("Visit workbench, then storage, and stop.")
    assert o.final_status == "rejected"
    assert "max steps" in o.final_message


# ---------------- LLM 模式（stub 客户端） ----------------

def test_llm_single_tool_call_then_final():
    client = StubLLM([
        tool_call("navigate_to", {"x": 0.0, "y": -3.2, "theta": 0.0}),
        final("task done"),
    ])
    planner, _ = make_planner(llm_client=client)
    o = planner.run("Go to storage.")
    assert o.final_status == "success"
    assert o.final_message == "task done"
    assert len(o.steps) == 1 and o.steps[0].result.success


def test_llm_rejects_out_of_workspace_call():
    # LLM 编造了工作区外的坐标 -> 校验拒绝 -> 中止计划
    client = StubLLM([
        tool_call("navigate_to", {"x": 99.0, "y": 0.0, "theta": 0.0}),
    ])
    planner, _ = make_planner(llm_client=client)
    o = planner.run("Go to nowhere.")
    assert o.final_status == "rejected"
    assert o.steps[0].recovery == "reject_step"


def test_llm_rejects_unknown_skill():
    client = StubLLM([
        tool_call("teleport", {"x": 0.0, "y": 0.0}),
    ])
    planner, _ = make_planner(llm_client=client)
    o = planner.run("Just teleport.")
    assert o.final_status == "rejected"
    assert o.steps[0].result.reason.value == "unknown_skill"


def test_llm_rejects_malformed_arguments():
    # LLM 返回非 dict 的 arguments（如 JSON 字符串）——规划器防御
    client = StubLLM([
        {"content": None,
         "tool_calls": [{"id": "call_1", "name": "navigate_to", "arguments": '{"x": 1.5}'}]},
    ])
    planner, _ = make_planner(llm_client=client)
    o = planner.run("Go somewhere.")
    assert o.final_status == "rejected"
    assert "not an object" in o.steps[0].result.details.get("message", "")


def test_llm_stops_after_max_steps():
    # LLM 一直输出工具调用 -> 步数超限被强制中止
    client = StubLLM([
        tool_call("navigate_to", {"x": 0.0, "y": -3.2, "theta": 0.0}, cid=f"c{i}")
        for i in range(10)
    ])
    planner, _ = make_planner(llm_client=client, max_steps=3)
    o = planner.run("Keep going.")
    assert o.final_status == "aborted"
    assert "max steps" in o.final_message


def test_llm_timeout_retry_once():
    # LLM 请求 charging_station，规划器超时短 -> 重试一次 -> 仍失败 -> 停止并报告
    client = StubLLM([
        tool_call("navigate_to", {"x": 2.2, "y": -3.5, "theta": 0.0}),
    ])
    planner, _ = make_planner(llm_client=client)
    o = planner.run("Go to charging_station.", default_timeout_s=0.8)
    assert o.final_status == "failed"
    assert o.steps[0].attempts == 2


# ---------------- 日志 ----------------

def test_system_prompt_workspace_matches_registry():
    # 确保系统提示词里的工作区边界与 skill_registry 的 WORKSPACE_LIMITS 一致
    planner, _ = make_planner()
    prompt = planner._system_prompt()
    assert "x in [-2, 4]" in prompt
    assert "y in [-5.2, 5.2]" in prompt
    assert "[-3, 3]" not in prompt


def test_planner_logs_full_record():
    log = "logs/test_planner.jsonl"
    if os.path.exists(log):
        os.remove(log)
    planner, _ = make_planner(log_path=log)
    planner.run("Go to storage.")
    with open(log, encoding="utf-8") as f:
        line = json.loads(f.readline())
    assert line["request"] == "Go to storage."
    assert line["original_plan"] != []
    assert line["steps"][0]["result"]["success"] is True
    assert line["final_status"] == "success"
    assert "elapsed_time_s" in line


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
