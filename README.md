# Leo Physics Engine

自研轻量物理引擎 + 类"人类一败涂地" ragdoll 游戏 + MCP 驱动的编辑器 AI agent。作为简历项目，三大卖点：自研 Verlet/PBD + RB-PBD 物理、ragdoll active 控制、引擎即 MCP server。

## 当前状态

- **M1 引擎骨架** ✅ — GLFW/OpenGL 4.6/entt/glm/glad2 + Blinn-Phong + flycam
- **M2 物理核心** ✅ — Verlet/PBD + RB-PBD 真刚体 + OBB-OBB SAT + ImGui 调试面板
- M3 ragdoll + 游戏（规划中）
- M4 编辑器 AI agent（规划中）
- M5 发布打包（规划中）

## 技术栈

| 层 | 技术 |
|---|---|
| 语言 | C++17（引擎）+ Python（AI/MCP，规划） |
| 构建 | CMake 3.25 + CMakePresets（VS2022 x64） |
| 窗口/输入 | GLFW + glad2（OpenGL 4.6 core） |
| 渲染 | OpenGL + Blinn-Phong + Debug 线框 |
| 数学 | glm |
| ECS | entt |
| UI/编辑器 | Dear ImGui |
| 物理 | 自研 Verlet/PBD（ragdoll）+ RB-PBD（刚体） |
| 测试 | GoogleTest |

依赖全部 CMake FetchContent，无需预装。

## 构建与运行

```powershell
# 配置 + 构建 (Release 推荐, Debug 物理慢)
cmake --preset vs2022-x64-release
cmake --build build/vs2022-x64-release --config Release --target leo_app

# 运行
.\build\vs2022-x64-release\bin\Release\leo_physics.exe
```

首次构建 FetchContent 下载 GLFW/glad2/glm/entt/ImGui/GoogleTest（约 100MB，需联网）。

## 单元测试

```powershell
cmake --build build/vs2022-x64-release --config Release --target leo_physics_tests
.\build\vs2022-x64-release\bin\Release\leo_physics_tests.exe
```

27 个测试：Verlet 积分 / 距离-角度约束 / 点-AABB 碰撞 / NaN 守卫 / 确定性回放 / RB-PBD（惯性张量、自由落体、四元数漂移、OBB SAT、偏心接触力矩、中心接触无力矩）。

## 操作说明（M2 demo）

| 按键 | 功能 |
|---|---|
| WASD | flycam 移动 |
| 鼠标 | 转视角（锁定模式） |
| Q / E | 上升 / 下降 |
| F | 在相机前方生成刚体箱子（掉落） |
| G | 切换 debug 线框显示（点/AABB/约束） |
| F1 | 切换鼠标锁定（flycam ↔ 点 ImGui） |
| ESC | 退出 |

ImGui 调试面板（F1 释放鼠标后可点）：重力 / 阻尼 / 子步数 / 迭代数 / debug 显示开关 + FPS / 物理耗时。

## 物理实现

### 双物理路线

- **Verlet/PBD（点-约束）** — 为 M3 ragdoll 准备：点质量 + 距离/角度约束 + PBD 迭代松弛。17 点人形 ragdoll 用。
- **RB-PBD（真刚体）** — M2 demo 箱子用：质心 + 四元数朝向 + 惯性张量 + 力矩。与 UE Chaos RB-PBD 同路线。

### RB-PBD 关键组件

- **RigidBody 组件**：质心 x、朝向 q(quat)、线/角速度 v/omega、反质量 + 反惯性张量（局部+世界）、半尺寸、休眠状态。
- **半隐式 Euler 积分**：`v += g·dt; x += v·dt; q += 0.5·quat(0,omega)·q·dt`，每帧 `inv_inertia_world = R·inv_inertia_local·Rᵀ`。
- **OBB-OBB SAT**：15 轴分离测试（各 3 局部轴 + 9 交叉轴），最小穿透轴为法线。
- **4 点接触流形**：面-面接触取 4 角点裁剪到对方 OBB，堆叠稳定（单点会跳变激发角速度）。
- **位置投影接触求解**：`w = inv_m + dot(r×n, inv_I_w·(r×n))`，`λ = C/w`，修正 `x += inv_m·λ·n` + `q += 0.5·quat(0, inv_I_w·(r×n)·λ)·q`。16 次迭代 + slop 容差。
- **冲量法速度求解**：法向速度归零（restitution=0）+ 库仑摩擦。用 collectContacts 缓存的流形（积分后/求解前），确保位置投影分离后仍施加冲量。
- **休眠机制**：速度低于阈值持续 0.3s → sleeping（跳过积分/求解，当静态处理），被动态接触唤醒。消除堆叠微抖。

### 翻倒（核心物理验证）

箱子搭在平台边缘、质心越过支撑面 → 重力产生绕边缘的力矩 → 翻转。这是 RB-PBD 力矩分配的效果，点云物理（无力矩）做不到。

## 项目结构

```
src/
├── core/           # 引擎核心
│   ├── Application/Window/Input/Camera    # 主循环 + flycam
│   ├── graphics/    # Renderer/Shader/Mesh/DebugRenderer
│   ├── scene/       # Scene(entt) + System + Components
│   └── ui/          # ImGuiLayer 调试面板
├── physics/        # 自研物理
│   ├── Verlet/Constraint/Collider         # 点-约束路线 (M3 ragdoll)
│   ├── RigidBody + RBIntegrate            # RB-PBD 刚体路线
│   └── PhysicsSystem                      # substep: integrate→collide→solve→sleep
├── editor/         # M4 占位 (MCP AI agent)
└── main.cpp        # demo 入口
tests/              # GoogleTest 27 个单测
docs/specs/         # 设计规格
assets/shaders/     # blinn_phong + debug
```

## 设计文档

完整设计规格见 [docs/specs/2026-07-02-physics-engine-design.md](docs/specs/2026-07-02-physics-engine-design.md)，含架构、物理设计、AI/MCP 协议、里程碑、测试策略、决策记录。

## 截图

M2 demo（RB-PBD 刚体箱子）：
- `docs/m2_falling.png` — 箱子从空中下落
- `docs/m2_stacking.png` — 落地堆叠
- `docs/m2_settled.png` — 休眠静止
