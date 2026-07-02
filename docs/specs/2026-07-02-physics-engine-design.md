# Leo Physics Engine — 设计规格

- 日期：2026-07-02
- 状态：修订中（v2，已纳入规格审查反馈）
- 作者：zian.zhou
- 时间预算：3-4 个月（约 14-18 周，含调参缓冲）

## 1. 项目定位

**Leo Physics Engine** —— 自研轻量物理引擎 + 类"人类一败涂地" ragdoll 小游戏 + MCP 驱动的编辑器 AI agent，作为简历项目。

三大卖点：
1. **自研 Verlet/PBD 物理引擎**（核心，差异化）
2. **ragdoll active 控制**（人类一败涂地手感）
3. **引擎即 MCP server**（AI 辅助关卡生成，当下热门）

核心判断：物理是主角，走 Jakobsen Verlet/PBD 路线（非 PhysX 冲量法/Featherstone 深坑），用约 1/5 的代码量做出"软乎乎"ragdoll 手感，3-4 个月可达成。

## 2. 总体架构

### 分层

```
┌─ AI 层 (Python, 仅编辑器模式) ────────────────────┐
│  LLM (Claude API, tool-use) → MCP Client           │
└───────────────┬────────────────────────────────────┘
                │ MCP (JSON-RPC over stdio)
┌───────────────▼────────────────────────────────────┐
│  MCP Server 层 (Python, mcp SDK)                   │
│  声明 tools, 转发为简单 JSON 指令, (AI-Feedback阶段)收状态反馈 │
└───────────────┬────────────────────────────────────┘
                │ 本地 socket + JSON
┌───────────────▼────────────────────────────────────┐
│  引擎核心 (C++)                                     │
│  ├─ ECS (entt): Transform/VerletPoint/Collider/... │
│  ├─ 物理: Verlet 积分 + PBD 约束求解器 + 子步进     │
│  ├─ 碰撞: 点 vs AABB, 静态刚体 + 动态点云箱(RB-PBD演进) │
│  ├─ 渲染: OpenGL + Blinn-Phong 基本光照            │
│  ├─ ragdoll: 17 点人形, C→A 演进控制               │
│  └─ 编辑器: Dear ImGui 面板 + socket 服务端        │
└────────────────────────────────────────────────────┘
                │ GLFW 窗口 + 输入
            玩家 (WASD/鼠标/抓握控制 ragdoll)
```

### 进程边界
- C++ 引擎一个进程，Python（MCP server + LLM）一个进程
- 本地 socket 通信，两边独立崩溃互不影响

### 模式切换（支撑发布）
- **编辑器模式**（开发期）：开 ImGui、开 socket 接 AI、可摆放/保存关卡
- **游戏模式**（玩家）：只读关卡、只跑游戏循环、关 ImGui/socket、全屏窗口
- 发布版编译为游戏模式

### 资源打包
- 关卡（.scene.json）+ shader 嵌入 exe（C++ 字节数组，构建脚本转 .h）
- 发布形态：单个 game.exe，玩家双击即玩，无需 Python/API key/额外文件

## 3. 物理引擎设计

### 3.1 物体表示（软硬分工）

| 物体 | 表示 | 硬度 |
|---|---|---|
| ragdoll（角色） | Verlet 点云 | 软（少约束、允许形变） |
| 地面/墙/平台 | 静态刚体 AABB | 硬（不动，无限质量） |
| 可推动箱子 | 硬点云 →（演进）RB-PBD 真刚体 | 硬 |

软硬对比即"人类一败涂地"核心手感：角色软乎乎、世界硬邦邦。

### 3.2 积分器：Verlet（位置法）

```
对每个点:
  v = (x_current - x_prev) * damping     # 速度隐含在位置差
  x_prev = x_current
  x_current = x_current + v + a * dt²     # a = 重力/外力
```

### 3.3 约束求解器：PBD 迭代松弛

**Constraint 接口**（所有约束统一实现）：

```cpp
struct Constraint {
  virtual void project(float dt) = 0;  // 把违反量投影回满足约束的位置
  virtual float violation() const = 0; // 当前违反量（调试/监控用）
};
// 子类: DistanceConstraint, AngleConstraint, TargetPoseSpring(Active阶段)
```

**求解器主循环**（每子步）：

```cpp
void PhysicsSystem::substep(float dt) {
  integrate(dt);                          // Verlet 积分 (重力/外力)
  for (int iter = 0; iter < N; ++iter) {  // N = 8~16
    resolveCollisions();                  // 先解碰撞穿透 (点 vs Collider)
    for (auto& c : constraints) c.project(dt);  // 再解距离/角度约束
  }
  applyDamping();                         // 位置阻尼
  detectNaN();                            // 守卫: NaN/Inf 检测 + 回滚
}
```

- 约束存储为 `std::vector<Constraint*>`，按类型分组（距离/角度/目标姿态），同组连续遍历利于缓存
- 投影顺序：碰撞 → 距离 → 角度 → 目标姿态（后者的弹簧力在积分阶段施加，投影阶段做位置修正）
- **子步进**：每帧拆 4-8 子步，每步执行上述循环，提升稳定性

### 3.4 ECS 组件划分

| Component | 字段 | 持有者 |
|---|---|---|
| `Transform` | pos, rot | 所有可见物体 |
| `VerletPoint` | x_current, x_prev, mass, collision_group | ragdoll 点、点云箱子点 |
| `StaticBody` | AABB, friction | 地面/墙/平台 |
| `ColliderRef` | 指向 Collider 接口实现 | 箱子（点云或 RB-PBD） |
| `Renderable` | mesh, shader uniforms | 所有需渲染物体 |
| `RagdollRef` | 点 entity 列表 + 约束列表 | 角色（一个 ragdoll = 一个 RagdollRef entity + N 个 VerletPoint entity） |
| `Light` | type, color, direction/pos | 光源 |

- Verlet 点作为独立 entity（SoA 友好，entt 自动按组件池连续存储），RagdollRef 持有点的 entity 列表
- 约束作为 RagdollRef 内的 `vector<Constraint>`，不单独成 entity（约束强属于某个 ragdoll）
- 箱子点云同理：一个箱子 entity 持有 ColliderRef（指向点云实现）+ 内部点列表

### 3.5 碰撞

- **点 vs 静态 AABB**：穿透点沿法线推出 + 切向摩擦（速度衰减）
- **点 vs 动态箱子**：经抽象 Collider 接口（见 3.7）
- **碰撞过滤**：相邻 ragdoll 点同 `collision_group`，不互相碰撞（防自碰撞爆炸）

### 3.6 稳定性手段（借鉴 UE 调研）

- 位置阻尼（damping 因子）
- 质量分配（骨盆重、四肢轻，影响约束位移分配）
- 子步进
- 求解顺序：先解碰撞穿透，再解距离/角度约束

### 3.7 Collider 抽象接口（演进关键）

```
Collider 接口:
  - get_aabb()                          → 包围盒（broadphase）
  - collide_point(p) → 接触点+法线+穿透  （ragdoll 点撞箱子）
  - 接受位置修正/冲量

阶段1 实现: 硬点云箱子 (8 点 + 12 条强距离约束, 复用 Verlet 求解)
阶段2 实现: RB-PBD 真刚体 (位置+旋转+惯性张量+朝向投影+接触约束)
```

ragdoll 点和地面只与 Collider 接口交互，不知道箱子内部是点云还是刚体。
**演进无痛前提**：接口从一开始就隔离，点云细节不泄漏到碰撞代码各处。

### 3.7 关于 RB-PBD 的认知

- Verlet 点 PBD 与 RB-PBD **同属 PBD 思想**（位置投影，非冲量法）
- 区别：投影对象不同——点 PBD 投影点位置；RB-PBD 投影刚体位置+朝向（涉及惯性张量逆）
- RB-PBD 真实收益在"多刚体精确堆叠/摩擦"，本游戏基本用不到
- 演进为可选项，时间余量足够才做，停在硬点云不影响发布

## 4. ragdoll 结构与玩家控制

### 4.1 骨骼结构（17 点人形）

```
        ● 头
        │
        ● 颈
   ┌────┼────┐
  上臂   │   上臂
   ●    ●躯干上 ●
   │    │      │
  前臂  ●躯干下 前臂
   ●    │      ●
   │    ● 骨盆  │
   ●手  ┌──┴──┐ ●手
        │     │
       大腿  大腿
        ●     ●
        │     │
       小腿  小腿
        ●     ●
        │     │
        ●脚   ●脚
```

- 17 点：头、颈、躯干上/下、骨盆、左右上臂/前臂/手、左右大腿/小腿/脚
- 约束：相邻点距离约束（保骨长），肘/膝/颈角度约束限位
- 质量分配：骨盆最重、躯干次之、四肢较轻、手脚最轻
- 碰撞过滤：相邻点同 `collision_group`

### 4.2 玩家控制两阶段演进

> 命名说明：放弃易混淆的 C/B/A 字母，ragdoll 控制用 `Ragdoll-Decorative` → `Ragdoll-Active`；AI 用 `AI-Intent` → `AI-Feedback`。下文同。

**Ragdoll-Decorative（先跑通）：伪移动 + 物理装饰**
- WASD → 给骨盆施加水平速度/位移（半伪物理）
- ragdoll 其余点靠约束跟着晃（物理装饰）
- 跌倒检测：骨盆高度 < 阈值 或 头朝下 → 切纯物理倒地
- 起身：按键把骨盆拉回站立目标位置
- **抓握最小版**：左右手能抓住固定锚点悬挂/攀爬一格（接触后建立吸附约束），不做完整攀爬
- 角色能动、能推箱子、能摔倒、能爬起来、能挂住；极稳

**Ragdoll-Active（演进）：active ragdoll 电机控制**
- WASD → 设定"迈步前倾"目标姿态（每根骨骼目标位置/朝向）
- 每点加目标姿态弹簧：PD 控制器（k=刚度，d=阻尼）
- 物理求解：弹簧力 + 重力 + 碰撞 + 约束
- 角色想站立/迈步，被撞/踩空会踉跄 → 人类一败涂地手感
- 完整攀爬：抓握 + 摆荡 + 翻越
- 关键参数：OrientationStrength、PositionStrength、阻尼、最大力（借鉴 UE PhysicalAnimationComponent）

### 4.3 输入映射

- WASD → 骨盆/整体目标移动方向
- 鼠标 → 上半身朝向（抓取瞄准）
- 空格 → 跳跃（骨盆向上冲量）
- 左右键 → 左右手抓握（手臂伸向目标 + 接触后吸附约束）
- 抓握吸附是攀爬核心：`Ragdoll-Decorative` 阶段做最小版（悬挂/攀爬一格），`Ragdoll-Active` 阶段做完整版

## 5. AI agent 与 MCP 协议

### 5.1 三层架构（仅编辑器模式）

```
① LLM 层 (Python, Claude API, tool-use)
   接收自然语言 → 决定调用哪个 tool
        ↓ MCP (JSON-RPC over stdio)
② MCP Server 层 (Python, mcp SDK)
   声明 tools schema, 转成简单 JSON 指令
   (AI-Feedback阶段) 收引擎状态回传 LLM 判断
        ↓ 本地 socket + JSON
③ 引擎层 (C++)
   socket 服务端, 执行场景操作, 维护场景, 返回状态
```

### 5.2 MCP tools 定义

| tool | 参数 | 说明 |
|---|---|---|
| `spawn_object` | type, position, size, rotation | 放单个物体 |
| `spawn_layout` | layout_type, count, anchor, params | 语义布局原语，透传给引擎解析 |
| `remove_object` | id | 删除物体 |
| `clear_scene` | — | 清空场景 |
| `save_scene` | name | 存关卡文件 (.scene.json) |
| `load_scene` | name | 加载关卡 |
| `query_scene` | — | (AI-Feedback 阶段) 返回所有物体 id/位置/尺寸 |
| `run_simulation` | duration | 同步加速跑物理 N 秒看效果（dt 压缩，超时上限 5s） |

- `layout_type`：`staircase` / `row`（M4 必做）；`grid` / `circle` 按需（YAGNI，默认不实现）
- `anchor`：`near_wall_left` / `center` / `on_platform`（词汇表 M4 细化）

### 5.3 空间语义解析器（AI-Intent 阶段工程核心）

**归属层：引擎侧（C++）**。理由：`near_wall_left` 等 anchor 解析依赖场景几何信息（墙在哪、平台在哪），只有引擎持有场景数据。

- 引擎收到 `spawn_layout` 原语后，由解析器模块翻译成具体坐标
- 解析器知道：墙在哪、阶梯怎么排（每级升高 0.5m、间距 0.6m）、留缝参数
- 内部逐个调 `spawn_object` 放置，返回生成的 object id 列表
- **MCP server 只透传 `spawn_layout` 原语，不做坐标展开**（修正 v1 矛盾）
- 是简历亮点之一

### 5.4 AI-Intent → AI-Feedback 演进

**AI-Intent（核心）**：意图级指令 + 空间语义解析器
- LLM 输出 `spawn_layout(staircase, 3, near_wall_left)`
- 引擎解析器转坐标，逐个放置，返回 object id 列表

**AI-Feedback（演进）**：文本状态反馈闭环
- 放完后 LLM 调 `query_scene()` 获取物体列表（JSON）
- LLM 检查："箱子离墙太远" → `remove` + 重新 `spawn_layout`
- 纯文本反馈，不接多模态视觉（省复杂度）
- 闭环：观察 → 判断 → 调整

### 5.5 关卡流转（衔接发布）

```
开发期:  AI 生成场景 → save_scene("level1") → level1.scene.json
发布版:  game.exe 启动 → 嵌入的 level1.scene.json → 游戏模式加载 → 玩家玩
```
AI 产出是关卡文件，发布版只读文件，AI 不进发布。

### 5.6 socket 协议（引擎 ↔ MCP server）

**帧格式**：每条消息 = 4 字节长度前缀（小端 uint32）+ JSON 正文。避免换行/边界歧义。

**命令表（MCP server → 引擎）**：

| cmd | 字段 | 说明 |
|---|---|---|
| `spawn` | objects[] | 放置物体列表（spawn_object 展开） |
| `spawn_layout` | layout_type, count, anchor, params | 语义布局原语（引擎解析） |
| `remove` | id | |
| `clear` | — | |
| `save` | name | |
| `load` | name | |
| `query` | — | 请求场景状态 |
| `run_sim` | duration | |

**响应（引擎 → MCP server）**：

```json
// 成功
{"status": "ok", "ids": [1, 2, 3], "objects": [{"id":1,"pos":[1,0,0],"size":[1,0.5,1]}]}
// 失败
{"status": "error", "code": "INVALID_PARAM", "message": "size must be positive"}
```

**错误码**：`INVALID_PARAM`（参数非法：size 负数、type 未知、位置穿墙）、`NOT_FOUND`（id 不存在）、`PARSE_ERROR`（JSON 畸形）、`SIM_TIMEOUT`（run_sim 超时）、`INTERNAL`（引擎内部错误）

**socket 生命周期**：
- 引擎启动时监听本地端口（仅编辑器模式）
- MCP server 连接后保持长连接
- 心跳：每 5s 一次 ping/pong，10s 无响应判定断开
- 引擎崩溃：MCP server 检测到连接断开，向 LLM 返回 `INTERNAL` 错误，LLM 可提示用户重启引擎
- 半截消息/畸形 JSON：丢弃并回 `PARSE_ERROR`，不崩溃

**物理守卫**（引擎侧）：每子步后检测 NaN/Inf，发现则回滚到上一帧状态 + 记录日志 + 通过 `query` 返回警告标记。

**样例**：

LLM → MCP server（tool call）:
```json
{"tool": "spawn_layout", "params": {"layout_type": "staircase", "count": 3, "anchor": "near_wall_left", "gap": 0.1}}
```
MCP server → 引擎（socket，透传原语）:
```json
{"cmd": "spawn_layout", "layout_type": "staircase", "count": 3, "anchor": "near_wall_left", "gap": 0.1}
```
引擎 → MCP server（解析后返回 id 列表）:
```json
{"status": "ok", "ids": [1, 2, 3]}
```

## 6. 技术栈

| 层 | 技术 |
|---|---|
| 语言 | C++（引擎）+ Python（AI/MCP） |
| 构建 | CMake |
| 窗口/输入 | GLFW + glad |
| 渲染 | OpenGL + Blinn-Phong shader |
| 数学 | glm |
| ECS | entt |
| UI/编辑器 | Dear ImGui |
| 物理 | 自研 Verlet/PBD（+ RB-PBD 演进） |
| AI 协议 | MCP（Python `mcp` SDK） |
| LLM | Claude API（tool-use） |
| 引擎↔AI | 本地 socket + JSON |
| 资源 | 嵌入 exe（C++ 字节数组） |

## 7. 里程碑（14-18 周，含调参缓冲）

### M1 · 引擎骨架（约 3 周）

- CMake + GLFW + OpenGL 窗口
- entt ECS 接入
- 基本渲染：立方体/平面 + Blinn-Phong
- 摄像机、输入
- **交付物**：带光照的立方体在窗口里

### M2 · 物理核心（约 3 周）

- Verlet 积分器
- PBD 约束求解器（距离/角度）+ 子步进 + Constraint 接口
- 静态刚体 AABB + 点 vs AABB 碰撞
- 硬点云箱子（Collider 接口隔离）
- Dear ImGui 调试面板（实时调参）
- shader/关卡资源嵌入的构建脚本骨架
- **性能基准**：N 个箱子 + 1 个 ragdoll 稳定 60fps（单帧 <16ms，物理占比目标 <50%）
- **交付物**：点云箱子在地面堆叠、可推动、稳定不爆炸

### M3a · ragdoll 结构 + 物理稳定（约 2 周）

- 17 点人形 ragdoll + 距离/角度约束 + 碰撞过滤
- ragdoll 与地面/箱子的碰撞稳定（不抖动不穿透）
- ImGui 实时调参（约束刚度/阻尼/质量）
- **交付物**：ragdoll 能在场景里被推倒、滚动、堆叠，物理稳定

### M3b · 控制 + 抓握最小版 + 关卡（约 2 周）

- Ragdoll-Decorative 伪移动控制（WASD/跳跃/摔倒/起身）
- **抓握最小版**：左右手能抓住固定锚点悬挂/攀爬一格
- 关卡序列化（save/load .scene.json）
- 第一个可玩关卡（含推箱子 + 一个攀爬点）
- **交付物**：控制软乎乎角色走动、推箱子、挂住攀爬点

### M3c · 调参缓冲（约 1 周）

- 专门留给物理稳定性调参、手感打磨、bug 修复
- PBD 高约束密度下的抖动/爆炸排查
- **交付物**：demo 稳定可演示

### M4 · AI agent AI-Intent 阶段（约 2 周）

- Python MCP server + tools 定义
- 引擎 socket 服务端（帧格式 + 错误码 + 心跳 + NaN 守卫）
- 空间语义解析器（staircase/row + anchor 词汇表）
- 自然语言 → 场景生成
- **交付物**：自然语言在编辑器生成关卡

### M5 · 发布 + 打磨（约 2 周）

- 游戏/编辑器模式切换
- 资源嵌入 exe（关卡 + shader）
- 打包 game.exe
- 关卡设计、UI、手感调参
- **交付物**：可发布双击即玩的 exe

### M6 · 演进（时间余量，约 1-3 周，按优先级择优）

1. **Ragdoll-Active**（电机控制 + 完整攀爬）← 首选，最影响手感
2. **AI-Feedback** 反馈闭环
3. RB-PBD 真刚体箱子

**M6 硬性 fallback**：若 Ragdoll-Active 做不完，至少保证 M3b 的 Ragdoll-Decorative + 抓握最小版可发布——游戏核心玩法（推箱子 + 攀爬一格）已成立，卖点不塌。

### 风险项（显式列出）

- **PBD 高约束密度抖动/爆炸**：17 点 ragdoll × 多约束 × 子步迭代，调参可能吃掉 M3c 整周。缓解：子步进 + 阻尼 + 约束求解顺序 + 降低迭代数优先稳后准。
- **Ragdoll-Active 调参不达预期**：active ragdoll 电机控制是项目最难部分，可能耗尽 M6 仍手感一般。缓解：M3b 先有 Decorative 兜底，Active 是锦上添花。
- **AI-Feedback 闭环价值有限**：LLM 对空间坐标的文本判断可能不准。缓解：若效果差，停在 AI-Intent 即可，简历叙事仍成立。

## 8. 测试策略

### 8.1 物理单元测试（C++，随 M2/M3a 同步）

- **距离约束**：投影后 `|x_a - x_b| - rest_length| < ε`（ε=1e-4）
- **角度约束**：投影后关节夹角落在限位区间内
- **质量分配**：重质量点位移 < 轻质量点位移（按反质量比）
- **子步进能量守恒**：无外力下多帧后动能漂移 < 阈值
- **点 vs AABB**：穿透修正方向沿法线、修正后无穿透
- **NaN 守卫**：极端输入（零质量、超大外力）触发回滚而非传播

### 8.2 确定性测试

- 固定种子 + 固定输入，多帧后场景状态可复现（利于 replay/debug/回归）
- CI 中跑"100 帧确定性回放"断言

### 8.3 MCP 协议契约测试（M4）

- 每个 tool 的 schema 与引擎 socket 命令的 round-trip
- 错误码覆盖：INVALID_PARAM / NOT_FOUND / PARSE_ERROR / SIM_TIMEOUT
- 帧格式：长度前缀 + JSON 解析正确性、半截消息丢弃

### 8.4 ragdoll 手感客观指标（调参基准）

- 站立稳定时骨盆高度方差（越小越稳）
- 推箱子后恢复站立时间
- 摔倒后起身成功率
- 这些指标写入 ImGui 调试面板，作为调参的客观参照（非自动断言）

## 9. 简历叙事

> **Leo Physics Engine** —— 自研轻量物理引擎：Verlet/PBD 求解器（与 UE5 Chaos 的 PBD 求解器同属位置投影思想）+ 抽象 Collider 接口（预留 RB-PBD 真刚体演进）。基于此实现类"人类一败涂地" ragdoll 游戏（17 点人形、active ragdoll 电机控制、抓握攀爬）。引擎作为 MCP server 暴露工具接口，LLM agent 在编辑器阶段用自然语言生成关卡（意图级指令 + 空间语义解析器 + 反馈闭环）。最终打包为单文件 exe 发布。


## 10. 关键设计决策记录

| 决策点 | 结论 | 理由 |
|---|---|---|
| 引擎命名 | Leo Physics Engine | — |
| 物理求解器路线 | Verlet/PBD 自研（非 PhysX 冲量法） | 避开刚体动力学深坑，3-4 月可达成，保留"自研物理"卖点 |
| 物体表示 | ragdoll 软点云 + 静态刚体地面 + 硬点云箱子 | 软硬对比即游戏手感，统一 Verlet 体系 |
| 箱子演进 | 硬点云 → RB-PBD（Collider 接口隔离） | 先跑通再升级，接口设计保演进无痛 |
| ragdoll 控制 | Ragdoll-Decorative → Ragdoll-Active 渐进 | 保证可演示 demo，再追手感 |
| ragdoll 结构 | 17 点标准人形 | 手感与复杂度甜点区 |
| 抓握 | Decorative 阶段做最小版（悬挂/攀爬一格），Active 阶段做完整版 | 攀爬是核心玩法，不能全推到演进阶段 |
| AI 角色 | 仅编辑器阶段，不进发布 | LLM 依赖 API/key，玩家无；AI 产出关卡文件供发布 |
| AI 粒度 | AI-Intent → AI-Feedback 渐进 | 空间语义解析器为核心亮点，文本反馈省复杂度 |
| 解析器归属 | 引擎侧（C++），MCP server 只透传原语 | anchor 解析需场景几何信息，只有引擎持有 |
| 通信架构 | Python MCP server + C++ socket | MCP 用 SDK 省力，C++ 不被协议污染，分层清晰 |
| 发布 | 游戏/编辑器模式 + 资源嵌入 exe | 单文件发布体验最佳 |
| 渲染/ECS | OpenGL + entt | 重点是物理非图形，资源最丰富起步最快 |
| 阶段命名 | 语义命名（Decorative/Active、Intent/Feedback），弃用易混字母 | 可读性，避免跨维度混淆 |

## 11. 待办与开放问题

- M6 三个演进项的具体取舍留待 M5 结束后根据余量决定
- 抓握吸附的具体约束实现（Ragdoll-Active 阶段）需在实现时细化
- 空间语义解析器的 anchor 词汇表需在 M4 细化

