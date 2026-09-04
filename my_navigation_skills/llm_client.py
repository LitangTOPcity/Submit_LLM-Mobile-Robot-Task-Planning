"""Part 4 —— OpenAI 兼容客户端（基于 requests 直连，不依赖 openai SDK）
=============================================================================

选型说明：openai 官方 SDK 依赖 httpx，在部分环境中与已安装的 httpx/httpx2
版本冲突（如 "TypeError: process() takes no keyword arguments" 或
"APIConnectionError"）。基于 requests 直连 REST 接口可避免该冲突，且兼容
所有 OpenAI 兼容服务：DeepSeek / OpenAI / Ollama 等。

环境变量：
    OPENAI_API_KEY   必填（或构造时传 api_key）
    OPENAI_BASE_URL  可选，默认 https://api.openai.com/v1；
                     DeepSeek 用 https://api.deepseek.com/v1
    LLM_MODEL        可选，默认 deepseek-chat

用法：
    client = OpenAIClient()
    resp = client.chat(messages, tools=tool_schemas())
    # resp == {"content": str | None,
    #          "tool_calls": [{"id": str, "name": str, "arguments": dict}] | None}

注意：不要把 API key 提交进仓库（任务书明确要求）。
"""
from __future__ import annotations

import json
import os
from typing import Any, Optional

import requests


class OpenAIClient:
    """把任意 OpenAI 兼容 API 的响应包装成规划器使用的简单 dict 协议。"""

    def __init__(self, model: Optional[str] = None,
                 base_url: Optional[str] = None,
                 api_key: Optional[str] = None):
        self.model = model or os.environ.get("LLM_MODEL", "deepseek-chat")
        self.base_url = (base_url or os.environ.get("OPENAI_BASE_URL")
                         or "https://api.openai.com/v1").rstrip("/")
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("缺少 API key：请设置环境变量 OPENAI_API_KEY 或传入 api_key 参数")

    def chat(self, messages: list[dict], tools: Optional[list[dict]] = None) -> dict[str, Any]:
        """发送 chat/completions 请求，返回 {"content": ..., "tool_calls": [...] | None}。"""
        payload: dict[str, Any] = {"model": self.model, "messages": messages}
        if tools:
            payload["tools"] = tools

        resp = requests.post(
            f"{self.base_url}/chat/completions",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            json=payload,
            timeout=120,
        )
        resp.raise_for_status()  # 401/404 等错误会在这里抛出，便于排查
        data = resp.json()

        msg = data["choices"][0]["message"]
        tool_calls = None
        raw_calls = msg.get("tool_calls")
        if raw_calls:
            tool_calls = []
            for tc in raw_calls:
                fn = tc.get("function", {})
                try:
                    args = json.loads(fn.get("arguments") or "{}")
                except json.JSONDecodeError:
                    args = {"_malformed_json": fn.get("arguments")}
                tool_calls.append({"id": tc.get("id"), "name": fn.get("name"),
                                   "arguments": args})

        return {"content": msg.get("content"), "tool_calls": tool_calls}
