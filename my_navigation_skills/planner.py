"""Part 4 —— 任务级规划器（LLM 工具调用 + 安全验证 + 反馈恢复）
=================================================================

把自然语言请求变成"经过验证的技能调用序列"并执行。

两种模式（共用同一套恢复/日志逻辑）：
  * Mock 模式（llm_client=None）：规则解析请求 -> 技能序列，
    模拟"LLM 输出的 tool call 计划"（无 API key 时可用）；
  * LLM 模式（llm_client=OpenAIClient()）：chat 循环，LLM 输出 tool_calls，
    规划器逐个执行（走 SkillExecutor 的 allowlist/校验/互斥），
    把每个技能的结构化结果回喂给 LLM，直到 LLM 输出最终答复或步数超限。

安全（任务书 Part 4 要求，全部落地）：
  - 固定 allowlist（只有 Part 2 的三个技能），不执行任何 LLM 生成的代码/命令；
  - 每个调用经 SkillExecutor 校验（未知技能 / 类型错误 / 越界 / 工作区）；
  - 计划步数 <= max_steps，重试 <= max_retries（有界，防止死循环）；
  - 非法位置直接拒绝，并提示合法位置。

恢复（任务书要求"至少一种有界响应"，纯规则即可）：
  - TIMEOUT            -> 重试一次；仍失败 -> 停止并报告；
  - INVALID_ARGUMENTS /
    UNKNOWN_SKILL      -> 拒绝该步并中止计划（如 LLM 编造技能/越界坐标）；
  - PRECONDITION_FAILED-> 拒绝该步并中止计划。

日志：每轮任务写一行 JSON（original_plan / 每步结果 / recovery 决策 / 最终结果）。
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Optional

from .locations import LOCATIONS, locations_for_prompt
from .models import Reason, SkillResult
from .skill_registry import WORKSPACE_LIMITS, SkillExecutor, tool_schemas


@dataclass
class StepRecord:
    """一步技能调用的完整记录（含恢复决策）。"""
    index: int
    skill: str
    arguments: dict
    result: SkillResult
    recovery: Optional[str] = None   # 例如 "retry_once" / "retry_once; stop_and_report" / "reject_step"
    attempts: int = 1


@dataclass
class TaskOutcome:
    """一轮自然语言任务的完整结果。"""
    request: str
    original_plan: list[dict] = field(default_factory=list)   # [{"skill":..., "arguments":...}, ...]
    steps: list[StepRecord] = field(default_factory=list)
    final_status: str = ""            # success | rejected | failed | aborted
    final_message: str = ""
    elapsed_time_s: float = 0.0
    mode: str = ""                    # "llm"（真实 function calling）| "mock"（规则解析）


class TaskPlanner:
    def __init__(self, executor: SkillExecutor,
                 llm_client: Any = None,
                 max_steps: int = 6,
                 max_retries: int = 1,
                 log_path: str = "logs/task_planner.jsonl",
                 verbose: bool = False):
        self.executor = executor
        self.llm_client = llm_client      # None = Mock 模式；否则需有 chat(messages, tools) -> dict
        self.max_steps = max_steps
        self.max_retries = max_retries
        self.log_path = log_path
        self.verbose = verbose

    # ============================ 对外入口 ============================

    def run(self, request: str, default_timeout_s: Optional[float] = None) -> TaskOutcome:
        start = time.monotonic()
        self._retries_left = self.max_retries
        outcome = TaskOutcome(request=request)
        outcome.mode = "llm" if self.llm_client is not None else "mock"

        if self.llm_client is None:
            self._run_mock(request, outcome, default_timeout_s)
        else:
            self._run_llm(request, outcome, default_timeout_s)

        outcome.elapsed_time_s = round(time.monotonic() - start, 3)
        self._log_outcome(outcome)
        return outcome

    # ============================ Mock 模式 ============================

    def _run_mock(self, request: str, outcome: TaskOutcome,
                  default_timeout_s: Optional[float]) -> None:
        """规则解析请求，生成"模拟的 LLM tool-call 计划"，然后走统一执行/恢复。"""
        plan, rejection = self._plan_mock(request)
        if rejection:
            outcome.final_status = "rejected"
            outcome.final_message = rejection
            return

        outcome.original_plan = [{"skill": s, "arguments": a} for s, a in plan]
        if len(plan) > self.max_steps:
            outcome.final_status = "rejected"
            outcome.final_message = f"plan exceeds max steps ({self.max_steps}): {len(plan)} calls"
            return

        for idx, (skill, args) in enumerate(plan):
            rec = self._execute_one(idx, skill, args, default_timeout_s, outcome)
            if not rec.result.success:
                if rec.recovery and "reject_step" in rec.recovery:
                    outcome.final_status = "rejected"
                    outcome.final_message = (f"step '{rec.skill}' rejected: "
                                             f"{rec.result.details.get('message', rec.result.reason.value)}")
                else:
                    outcome.final_status = "failed"
                    outcome.final_message = (f"cannot reach target: '{rec.skill}' "
                                             f"({rec.result.reason.value}) after {rec.attempts} attempt(s)")
                return

        outcome.final_status = "success"
        outcome.final_message = "all steps completed"

    def _plan_mock(self, request: str) -> tuple[list[tuple[str, dict]], Optional[str]]:
        """规则版"LLM 工具调用输出"：识别请求中出现的位置名，按出现顺序规划。

        支持任务书要求的两种请求：
          "Go to <location>."
          "Visit <A>, then <B>, and stop."
        以及非法请求（位置名不在表里 -> 拒绝并列出合法位置）。
        """
        text = request.lower()
        found = [name for name in LOCATIONS if name in text]
        found.sort(key=text.find)  # 按在请求中出现的先后排序
        if not found:
            return [], (f"unknown location in request \"{request}\"; "
                        f"valid locations: {list(LOCATIONS)}")

        plan: list[tuple[str, dict]] = []
        for name in found:
            pose = LOCATIONS[name]
            plan.append(("navigate_to", {"x": pose["x"], "y": pose["y"], "theta": pose["theta"]}))
        if any("stop" in word for word in text.split()):
            plan.append(("stop", {}))
        return plan, None

    # ============================ LLM 模式 ============================

    def _run_llm(self, request: str, outcome: TaskOutcome,
                 default_timeout_s: Optional[float]) -> None:
        """chat 循环：LLM 输出 tool_calls -> 逐个执行并回喂结果 -> 直到最终答复。"""
        messages = [
            {"role": "system", "content": self._system_prompt()},
            {"role": "user", "content": request},
        ]
        total_calls = 0

        for _ in range(self.max_steps):
            resp = self.llm_client.chat(messages, tools=tool_schemas())
            tool_calls = resp.get("tool_calls")

            if not tool_calls:  # LLM 输出最终答复
                outcome.final_message = resp.get("content") or ""
                outcome.final_status = "rejected" if not outcome.steps else "success"
                return

            # 把 LLM 这一轮的 tool_calls 追加为 assistant 消息
            messages.append({
                "role": "assistant",
                "content": None,
                "tool_calls": [
                    {"id": tc["id"], "type": "function",
                     "function": {"name": tc["name"], "arguments": json.dumps(tc["arguments"])}}
                    for tc in tool_calls
                ],
            })

            for tc in tool_calls:
                total_calls += 1
                if total_calls > self.max_steps:  # 有界：步数上限
                    outcome.final_status = "aborted"
                    outcome.final_message = f"exceeded max steps ({self.max_steps})"
                    return

                rec = self._execute_one(total_calls - 1, tc["name"],
                                        tc.get("arguments"), default_timeout_s, outcome)
                outcome.original_plan.append({"skill": rec.skill, "arguments": rec.arguments})
                messages.append({"role": "tool", "tool_call_id": tc["id"],
                                 "content": rec.result.to_json()})

                if not rec.result.success:
                    if rec.recovery and "reject_step" in rec.recovery:
                        outcome.final_status = "rejected"
                        outcome.final_message = (f"step '{rec.skill}' rejected: "
                                                 f"{rec.result.details.get('message', rec.result.reason.value)}")
                        return
                    outcome.final_status = "failed"
                    outcome.final_message = (f"cannot reach target: '{rec.skill}' "
                                             f"({rec.result.reason.value}) after {rec.attempts} attempt(s)")
                    return

        outcome.final_status = "aborted"
        outcome.final_message = f"exceeded max steps ({self.max_steps})"

    # ============================ 执行 + 恢复（两种模式共用） ============================

    def _execute_one(self, index: int, skill: str, args: Any,
                     timeout_s: Optional[float], outcome: TaskOutcome) -> StepRecord:
        # 防御：LLM 可能返回非 dict 的 arguments（如 JSON 字符串）
        if not isinstance(args, dict):
            result = SkillResult(skill=skill, success=False, reason=Reason.INVALID_ARGUMENTS,
                                 elapsed_time_s=0.0,
                                 details={"message": f"arguments not an object: {args!r}"})
            rec = StepRecord(index=index, skill=skill, arguments=args, result=result,
                             recovery="reject_step", attempts=1)
            outcome.steps.append(rec)
            return rec

        result = self.executor.execute(skill, args, timeout_s=timeout_s)
        actions: list[str] = []
        attempts = 1

        if not result.success:
            decision = self._recover(skill, args, result, timeout_s)
            if decision == "retry_once":
                actions.append("retry_once")
                result = self.executor.execute(skill, args, timeout_s=timeout_s)
                attempts = 2
                if not result.success:
                    actions.append("stop_and_report")  # 重试仍失败：停止并报告
            elif decision == "reject_step":
                actions.append("reject_step")
            else:
                actions.append("stop_and_report")

        rec = StepRecord(index=index, skill=skill, arguments=args, result=result,
                         recovery="; ".join(actions) if actions else None,
                         attempts=attempts)
        outcome.steps.append(rec)
        return rec

    def _recover(self, skill: str, args: dict, result: SkillResult,
                 timeout_s: Optional[float]) -> str:
        """有界恢复决策（纯规则，满足任务书"至少一种有界响应"）。"""
        if result.reason == Reason.TIMEOUT and self._retries_left > 0:
            self._retries_left -= 1
            return "retry_once"
        if result.reason in (Reason.INVALID_ARGUMENTS, Reason.UNKNOWN_SKILL,
                             Reason.PRECONDITION_FAILED):
            return "reject_step"
        return "stop_and_report"

    # ============================ 提示词与日志 ============================

    def _system_prompt(self) -> str:
        # 工作区边界从 skill_registry.WORKSPACE_LIMITS 动态读取，避免硬编码不一致
        wl = WORKSPACE_LIMITS
        return (
            "You are a task planner for a mobile robot in a simulated laboratory.\n"
            "The robot can ONLY use the following skills (strict allowlist):\n"
            f"{json.dumps(tool_schemas(), ensure_ascii=False)}\n"
            "Known locations (use ONLY these coordinates):\n"
            f"{locations_for_prompt()}\n"
            f"Workspace bounds: x in [{wl['x_min']:g}, {wl['x_max']:g}], "
            f"y in [{wl['y_min']:g}, {wl['y_max']:g}]. "
            "Never propose targets outside this range.\n"
            "Rules:\n"
            "1. If the user asks for a location NOT in the list, do NOT call any tool; "
            "reply exactly \"invalid location: <name>\".\n"
            "2. Choose skills from the allowlist only; never propose velocity commands or code.\n"
            f"3. Keep the plan to at most {self.max_steps} tool calls.\n"
            '4. If the user says "stop" at the end, include stop() as the final call.'
        )

    def _log_outcome(self, outcome: TaskOutcome) -> None:
        line = {
            "request": outcome.request,
            "mode": outcome.mode,
            "original_plan": outcome.original_plan,
            "steps": [
                {
                    "index": s.index,
                    "skill": s.skill,
                    "arguments": s.arguments,
                    "attempts": s.attempts,
                    "recovery": s.recovery,
                    "result": s.result.to_dict(),
                }
                for s in outcome.steps
            ],
            "final_status": outcome.final_status,
            "final_message": outcome.final_message,
            "elapsed_time_s": outcome.elapsed_time_s,
        }
        if self.verbose:
            print(json.dumps(line, ensure_ascii=False, indent=2))
        if self.log_path:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(line, ensure_ascii=False) + "\n")
