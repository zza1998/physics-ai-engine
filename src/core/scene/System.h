#pragma once

namespace leo {

class Scene;

// System 接口: M2 PhysicsSystem 继承此类, 通过 Application::add_system 注册
// 主循环按注册顺序调用 update, 物理系统应注册在渲染前
class System {
public:
    virtual ~System() = default;
    virtual void update(float dt, Scene& scene) = 0;
    virtual const char* name() const = 0;
};

} // namespace leo
