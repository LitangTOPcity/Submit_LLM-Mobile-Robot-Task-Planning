# LLM-Assisted Mobile Robot Task Planning

本仓库为作业的代码提交：基于 Part 1/2 的环境与基础导航，实现了 **Part 3（结构化技能接口）**
和 **Part 4（LLM 任务规划）**，并在 Webots 仿真中完成验证。

> 提交主体为 `my_navigation_skills/`（Part 2/3/4 源码）+ 报告 + 结果文档。

## 目录结构

```
Submit/                       # 本仓库根目录
├── my_navigation_skills/     # Part 2–4 源码（8 个 Python 模块 + __init__.py）
├── tests/                    # 离线自动化测试（test_my_navigation.py / test_planner.py）
├── report/
│   ├── Report.pdf            # 技术报告（≤4 页双栏，含架构图）
│   └── report.tex            # 报告 LaTeX 源文件
├── docs/                     # 运行日志与证据（env_info.txt 任务日志 / nav_trace.txt 导航轨迹）
├── README.md                 # 本文件
├── RESULTS.md                # 测试指令结果汇总 + 示例日志
├── architecture.svg          # 架构图
├── requirements.txt          # 依赖（仅 LLM 模式需要 requests）
├── vm_run_part4.py           # Webots 仿真复现：Part 4 三场景
└── video/                    # 演示视频（本地保留，不入库；见"演示视频"）
```

## 快速开始（离线自检，无需 ROS/Webots）

> 视频中的演示均为 **Webots 仿真运行**（见下文"在虚拟机上运行"）。下面的离线自检
> 只是不用开 Webots 也能验证接口逻辑的便捷方式（MockRobot 运动学模拟）。

在仓库根目录（`my_navigation_skills/` 的上一级）下，用 bash 执行：

```bash
# 接口自检：成功 / 越界被拒 / 未知技能被拒
python3 - <<'EOF'
from my_navigation_skills.skill_registry import SkillExecutor
from my_navigation_skills.mock_robot import MockRobot
ex = SkillExecutor(MockRobot(time_scale=10.0), verbose=True)
print(ex.execute("navigate_to", {"x": 0.0, "y": -3.2, "theta": 0.0}).to_json())  # 成功
print(ex.execute("navigate_to", {"x": 10.0, "y": 1.0, "theta": 0.0}).to_json())  # 越界被拒
print(ex.execute("teleport", {"x": 0.0}).to_json())                               # 未知技能被拒
EOF

# 离线自动化测试（Part 3 接口 / Part 4 规划器）
python3 tests/test_my_navigation.py
python3 tests/test_planner.py
```

## 依赖（requirements.txt）

```bash
pip install -r requirements.txt    # 仅真实 LLM 模式（Part 4）需要 requests
```

- **Part 2/3 与 Mock 模式**：零第三方依赖（纯 Python 标准库）；
- **真实 LLM 模式**：需要 `requests`（`llm_client.py` 直连 OpenAI 兼容 API，不依赖 openai SDK，避免 httpx 版本冲突）。

## 在虚拟机上运行（ROS 2 + Webots）

```bash
ros2 launch webots_ros2_robomaster robot_launch.py
# 另一个终端，先确保 my_navigation_skills 可导入：
python -c "from my_navigation_skills.skill_registry import SkillExecutor; \
           from my_navigation_skills.nav_skills import NavigationSkills; \
           import rclpy; rclpy.init(); \
           ex = SkillExecutor(NavigationSkills(), verbose=True); \
           print(ex.execute('navigate_to', {'x': 1.0, 'y': 0.0, 'theta': 0.0}).to_json())"
```

## Part 1：环境搭建与手动控制（复现步骤）

本项目的仿真环境与 Part 1 手动控制可按下述步骤复现（对应任务书 Part 1）：

1. **安装环境**（Ubuntu 22.04）：
   ```bash
   sudo apt install ros-humble-teleop-twist-keyboard   # 键盘手动控制包
   ```
   ROS 2 Humble 与 Webots 的安装见各自的官方文档（参考文献见报告）。

2. **启动仿真**（终端 A）：
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch webots_ros2_robomaster robot_launch.py
   ```
   > 本项目采用"功能等效"环境：Webots + TurtleBot3 房子世界（参考包为
   > `webots_ros2_robomaster`，按实际可启动命令为准）。机器人出生在 odom 原点
   > `(0, 0, 0 rad)`。

3. **键盘手动控制**（终端 B）：
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   按键：`i/` 前进后退、`j/l` 左右转、`k` 停、`space` 急停。
   若机器人无反应，用 `ros2 topic info /cmd_vel` 确认话题订阅正常。

## 坐标约定（评分表要求"clear coordinate conventions"）

- 参考系：`odom`（世界系）；**x 向前、y 向左，theta 从 +x 轴逆时针为正**；
- 单位：位置为米（m），角度为弧度（rad）；
- 成功容差（`navigate_to`）：位置误差 ≤ 0.1 m **且** 朝向误差 ≤ 0.1 rad；
- 工作区：`x ∈ [-2, 4]`、`y ∈ [-5.2, 5.2]`（`WORKSPACE_LIMITS`，= 实际房间范围，
  从 TurtleBot3 房子世界测得：西墙 x=-1.36、东墙 x=3.74、南墙 y=-5.1、北墙 y=5.1）；
- 命名位置（`locations.py`）按真实布局选择，各点与障碍物间距 ≥ 0.5 m、
  任意两点直线路径不穿家具；
- 同一套数值同时写在 `nav_skills.py`（实现）和 `skill_registry.py`（契约）里，
  测试 `test_registry_tolerances_match_success_condition` 保证二者一致。

## Part 2/3 对照评分表自查清单

| 评分点 | 对应实现 | 状态 |
|---|---|---|
| Part2 正确到达：位置+朝向容差内才算成功 | `navigate_to` 两阶段状态机，`heading_error` vs `target_theta` | ✅ 已在 Webots 验证 |
| Part2 超时安全 | 超时即 `stop()` 并返回 `reason=timeout` + 误差指标 | ✅ |
| Part2 follow_waypoints 报告最后到达点 | 返回 `last_reached` / `failed_at` | ✅ |
| Part2 控制逻辑可读 | `drive` / `rotate` 两阶段 + 注释 + `turn_command` | ✅ |
| Part3 契约五要素 | `SKILL_REGISTRY`：description / parameters / preconditions / success_condition / timeout | ✅ |
| Part3 带类型参数与范围 | `validate_arguments`（类型/范围/工作区/waypoints 嵌套） | ✅ |
| Part3 结构化结果 | `SkillResult`：success / reason（枚举）/ elapsed_time_s + 误差指标 | ✅ |
| Part3 紧凑注册表（给 LLM） | `compact_registry()` + `tool_schemas()`（OpenAI 格式）+ `json_schemas()`（标准 JSON Schema） | ✅ |
| Part3 模块化 | 实现（nav_skills）与接口（registry/executor）解耦，可离线测试 | ✅ |

> 验证结果：4 个命名位置全部到达、误差 ≤ 0.1（见 RESULTS.md）。

## 实现要点与设计取舍

| 设计决策 | 说明 |
|---|---|
| 成功判定：位置 + 朝向双容差 | `navigate_to` 只有在位置误差 ≤ 0.1 m **且** 朝向误差 ≤ 0.1 rad（相对 `target_theta`）时才报告成功，另设"到位后原地转正"阶段 |
| 超时处理 | 超时即 `stop()` 并返回 `reason=timeout`，同时带 `position_error_m` / `heading_error_rad` 指标 |
| 结果规范化 | `reason` 用枚举（OK/TIMEOUT/INVALID_ARGUMENTS/…）；每次调用返回结构化 JSON，Part 4 恢复逻辑据此决策 |
| 并发保护 | `SkillExecutor` 互斥：机器人忙时拒绝第二次调用（`stop` 例外，允许打断），避免互相覆盖 `/cmd_vel` |
| 结构化日志 | 每次调用写一行 JSON（`logs/skill_calls.jsonl`），供复盘与 RESULTS 汇总 |
| 给 LLM 的格式 | `tool_schemas()`（OpenAI function-calling）+ `compact_registry()`（提示词），与校验器共用同一份契约 |
| follow_waypoints 超时分配 | 总超时按 waypoint 数均分，避免"第一个点耗光全部预算" |
| 失败报告 | 成功/失败都返回 `last_reached`，失败追加 `failed_at` |
| 停车可靠性 | `stop()` 连发三次零速，确保仿真机器人停下 |

## Part 4：LLM 任务规划

两种模式，统一入口 `vm_run_part4.py`（在虚拟机上运行）：

```bash
# 在 Webots 仿真运行时（虚拟机内）：
cd <my_navigation_skills/ 所在的目录>        # 例如 /home/jerry/webots_ws/src

# Mock 模式（无 API key，任务书三类请求 + 恢复/安全场景，规则解析）
python3 vm_run_part4.py

# LLM 模式（真实 function calling；OpenAI / DeepSeek / Ollama 均可）
export OPENAI_API_KEY=sk-...
# 可选：export OPENAI_BASE_URL=https://api.deepseek.com/v1   export LLM_MODEL=deepseek-chat
python3 vm_run_part4.py
```

规划器 `TaskPlanner` 的设计（对照任务书 Part 4 要求）：

| 任务书要求 | 实现 |
|---|---|
| 自然语言 → 技能序列 | `TaskPlanner.run(request)`；Mock 模式规则解析，LLM 模式 chat 循环 + `tool_schemas()` |
| 支持三类请求 | 单点 / 多点并停止 / 非法位置（`Go to kitchen.` → rejected 并列出合法位置） |
| 严格 allowlist | `SkillExecutor` 只认注册表 3 个技能；不执行 LLM 生成的代码/命令 |
| 拒绝：未知技能/参数错误/越界 | `validate_arguments` + 工作区校验（`x=99` 这类坐标直接拒绝） |
| 计划步数/重试上限 | `max_steps` / `max_retries`（超限 → rejected / aborted，防死循环） |
| 反馈与恢复（有界） | `TIMEOUT` → 重试一次 → 仍失败 → 停止并报告；非法步骤 → 拒绝并中止 |
| 日志：原计划/结果/恢复决策/最终结果 | 每轮任务一行 JSON（`logs/task_planner.jsonl`） |

## 测试

接口与规划器配有离线自动化测试（`tests/`），运行：
`python tests/test_my_navigation.py` 与 `python tests/test_planner.py`，全部通过。

## 提交物清单（任务书 Code Submission 要求）

| 提交物 | 位置 | 状态 |
|---|---|---|
| 源码：导航技能 / 注册表接口 / 规划器 / 校验器 / 恢复逻辑 | `my_navigation_skills/` | ✅ |
| README（环境、依赖、启动命令、复现步骤） | 本文件 | ✅ |
| 架构图（LLM → validator → skills → 执行反馈路径） | `architecture.svg` | ✅ |
| 示例执行日志（一次成功 + 一次失败） | `RESULTS.md` 第 2 节 | ✅ |
| `RESULTS.md`（每指令：结果/耗时/调用数/恢复动作） | `RESULTS.md` | ✅ |
| 技术报告（4 页双栏 PDF，含架构图） | `report/Report.pdf`（源文件 `report/report.tex`） | ✅ |
| 演示视频（链接） | 见下方"演示视频"小节 | ✅ 已上传 |

## 演示视频

- 视频文件（本地保留，不入库）：`video/Submit_video_v2.mp4`
- 视频链接：https://www.bilibili.com/video/BV1ACtz6sEKY/
- 时长约 2 分 07 秒，含字幕与中文旁白，覆盖 Part 1（环境与手动控制）至 Part 4（LLM 规划）全流程。

## 注意事项

- API key 只通过环境变量传入，不写入代码或仓库（任务书明确要求）；
- `architecture.svg` 可直接嵌入 README / 报告，或转成 PDF/PNG 使用；
- 工作区边界 `WORKSPACE_LIMITS`（x∈[-2,4]、y∈[-5.2,5.2]）按 Webots 房间实测，更换仿真世界需重新测量；
- 运行日志（`logs/`）、虚拟环境（`.venv/`）、缓存（`__pycache__/`）均已加入 `.gitignore`，不入库。
