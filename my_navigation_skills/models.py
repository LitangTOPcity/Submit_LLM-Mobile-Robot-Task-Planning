"""Part 3 —— 结构化结果类型
============================

任务书要求每个技能调用返回结构化结果，至少包含 success / reason / elapsed_time。
用 dataclass 定义统一的 SkillResult，reason 用枚举表示，Part 4 的恢复逻辑据此
写规则分支（如 TIMEOUT → 重试一次、INVALID_ARGUMENTS → 拒绝、UNKNOWN_SKILL →
拒绝整条计划）。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class Reason(str, Enum):
    OK = "ok"
    UNKNOWN_SKILL = "unknown_skill"
    INVALID_ARGUMENTS = "invalid_arguments"
    PRECONDITION_FAILED = "precondition_failed"
    TIMEOUT = "timeout"
    INTERRUPTED = "interrupted"
    INTERNAL_ERROR = "internal_error"


@dataclass
class SkillResult:
    """一次技能调用的结构化结果。

    details 放可观测的量化指标（position_error_m / heading_error_rad /
    last_reached 等），Part 4 的反馈与恢复决策完全依赖它。
    """
    skill: str
    success: bool
    reason: Reason
    elapsed_time_s: float
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["reason"] = self.reason.value  # 枚举序列化为字符串
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)
