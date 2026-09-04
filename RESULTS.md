# RESULTS

> 每个测试指令的结果汇总：成功/失败、耗时、技能调用数、恢复动作。
> 数据来源：
> 1. **真实仿真** —— Webots（webots_ros2_robomaster）+ ROS 2 Humble，虚拟机运行（`ros2 run my_navigation_skills ...`）；
> 2. **离线验证** —— MockRobot（与真实节点同接口的运动学模拟），见 `tests/`（`test_my_navigation.py` / `test_planner.py`）。

## 1. Part 2 导航技能测试（4 个命名位置，Webots 仿真，对应报告 Table 1）

| # | 目标 | 结果 | 耗时 (s) | pos err (m) | hdg err (rad) |
|---|------|------|---------|------------|--------------|
| 1 | entrance（-0.7, -1.6, π） | 成功 | 17.4 | 0.086 | 0.099 |
| 2 | storage（0.0, -3.2, 0） | 成功 | 17.5 | 0.087 | 0.090 |
| 3 | workbench（1.5, -2.8, π/2） | 成功 | 14.9 | 0.086 | 0.086 |
| 4 | charging_station（2.2, -3.5, 0） | 成功 | 13.8 | 0.084 | 0.096 |
| 5 | 非法参数 `navigate_to(x=10.0, ...)` | 拒绝 | 0.0 | — | — |
| 6 | 未知技能 `teleport` | 拒绝 | 0.0 | — | — |
| 7 | 超时场景（timeout 0.8s） | 失败 | 0.8 | — | — |

> 1–4 为 4 个命名位置逐一到达（原始轨迹与结果见 `docs/nav_trace.txt`，
> 每点均 ≤ 0.1 m / ≤ 0.1 rad；workbench 的 θ=π/2 验证了"到位后原地转正"）。
> 5–6 为校验器拦截（未触达机器人），7 为恢复逻辑验证（重试一次 → 停止并报告，
> MockRobot 离线）。任务级结果见第 4 节。

## 2. 示例执行日志

### 成功示例（规则/Mock 模式，Webots 仿真真实运行）

```json
{"request": "Visit workbench, then storage, and stop.", "original_plan": [{"skill": "navigate_to", "arguments": {"x": 1.5, "y": -2.8, "theta": 1.57}}, {"skill": "navigate_to", "arguments": {"x": 0.0, "y": -3.2, "theta": 0.0}}, {"skill": "stop", "arguments": {}}], "steps": [{"index": 0, "skill": "navigate_to", "success": true, "reason": "ok", "elapsed_time_s": 14.096, "details": {"position_error_m": 0.087, "heading_error_rad": 0.099}}, {"index": 1, "skill": "navigate_to", "success": true, "reason": "ok", "elapsed_time_s": 17.044, "details": {"position_error_m": 0.098, "heading_error_rad": 0.093}}, {"index": 2, "skill": "stop", "success": true, "reason": "ok", "elapsed_time_s": 0.0, "details": {}}], "final_status": "success", "elapsed_time_s": 31.295}
```

### 失败示例 1（导航超时 → 重试一次 → 停止并报告，真实日志）

```json
{"request": "Go to storage.", "original_plan": [{"skill": "navigate_to", "arguments": {"x": 1.5, "y": 0.0, "theta": 0.0}}], "steps": [{"index": 0, "skill": "navigate_to", "arguments": {"x": 1.5, "y": 0.0, "theta": 0.0}, "attempts": 2, "recovery": "retry_once; stop_and_report", "result": {"skill": "navigate_to", "success": false, "reason": "timeout", "elapsed_time_s": 20.175, "details": {"position_error_m": 1.5, "heading_error_rad": 0.0}}}], "final_status": "failed", "final_message": "cannot reach target: 'navigate_to' (timeout) after 2 attempt(s)", "elapsed_time_s": 40.347}
```

> 说明：机器人未能到达目标（20.2 s 超时，位置误差 1.5 m）。规划器按规则
> 重试一次，仍失败后执行 `stop_and_report`——安全停车并返回结构化失败结果，
> 对应 `docs/env_info.txt` 真实日志（该次运行的目标为 storage 的早期坐标
> (1.5, 0.0)，与最终坐标 (0.0, -3.2) 不同；恢复逻辑与最终版本一致）。

### 失败示例 2（LLM 输出不完整参数 → 校验器拦截，真实 LLM 模式）

```json
{"request": "Visit workbench, then storage, and stop.", "mode": "llm",
 "original_plan": [{"skill": "follow_waypoints", "arguments": {"waypoints": [[1.5, -2.8], [0.0, -3.2]]}}],
 "steps": [{"index": 0, "skill": "follow_waypoints", "attempts": 1, "recovery": "reject_step",
            "result": {"success": false, "reason": "invalid_arguments",
                       "details": {"message": "waypoint[0] must be [x, y, theta]; waypoint[1] must be [x, y, theta]"}}}],
 "final_status": "rejected",
 "final_message": "step 'follow_waypoints' rejected: waypoint[0] must be [x, y, theta]; waypoint[1] must be [x, y, theta]",
 "elapsed_time_s": 0.943}
```

> 说明：LLM 生成的 waypoints 缺少 `theta` 字段，校验器当场拦截
> （`invalid_arguments` → `reject_step`），机器人未被执行——这正是任务书要求的
> "安全拒绝非法调用"。waypoints 使用 `{x, y, theta}` 对象格式后，同一指令由
> LLM 成功执行（见第 9L 行）。

## 3. 恢复行为演示（离线验证）

规划器对超时的有界响应（任务书 Part 4 要求）：
`TIMEOUT` → 重试一次（`attempts=2`）→ 仍失败 → `stop_and_report`。
对应 JSONL 日志见 `logs/skill_calls.jsonl` 与 `logs/task_planner.jsonl`（运行时生成）。

## 4. Part 4 任务级规划（Webots 仿真验证通过，Mock + 真实 LLM 双模式）

| # | 模式 | 指令 | 结果 | 耗时 (s) | 技能调用数 | 恢复动作 | 备注 |
|---|------|------|------|---------|-----------|---------|------|
| 8M | Mock | "Go to storage." | 成功 | 24.53 | 1 | 无 | 单点导航 |
| 9M | Mock | "Visit workbench, then storage, and stop." | 成功 | 31.30 | 3 | 无 | navigate×2 + stop，全部到达 |
| 10M | Mock | "Go to kitchen." | 拒绝 | 0.0 | 0 | 拒绝非法位置 | 未触达机器人，列出合法位置 |
| 8L | LLM | "Go to storage." | 成功 | 26.54 | 1 | 重试一次 | 超时后 `retry_once` 成功（pos_err 0.087） |
| 9L | LLM | "Visit workbench, then storage, and stop." | 成功 | 34.43 | 2 | 无 | LLM 选 follow_waypoints（对象格式 waypoints）+ stop |
| 10L | LLM | "Go to kitchen." | 拒绝 | 0.46 | 0 | 拒绝非法位置 | LLM 自主拒绝："invalid location: kitchen" |
| 11 | 离线 | 超时场景（timeout 0.8s） | 失败 | 0.8 | 2 次尝试 | `retry_once; stop_and_report` | 有界恢复已由规划器测试覆盖 |

> 数据来源：Webots 仿真（虚拟机内运行，`python3 vm_run_part4.py`）。
> 8M–10M 为规则/Mock 模式（不设 API key），8L–10L 为真实 LLM 模式。
> LLM 模式使用 DeepSeek（deepseek-chat，OpenAI 兼容 function calling）。
> 原始日志：`logs/task_planner.jsonl`。

复现方式：

```bash
# 方式 1：Webots 仿真（RESULTS 表 8M–10L 数据来源，规则 + 真实 LLM 双模式）
cd <my_navigation_skills/ 所在目录>
python3 vm_run_part4.py                       # 不设 key → Mock（规则解析）模式
export OPENAI_API_KEY=sk-...
python3 vm_run_part4.py                       # 设 key → 真实 LLM function calling

# 方式 2：离线快速验证（无 ROS/Webots/API key，覆盖三类请求 + 拒绝 + 恢复逻辑）
python3 tests/test_planner.py
python3 tests/test_my_navigation.py
```
