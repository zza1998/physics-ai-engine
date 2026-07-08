#include "physics/RBIntegrate.h"
#include "physics/PhysicsTypes.h"
#include <glm/gtc/quaternion.hpp>

namespace leo {

void integrateRigidBodies(entt::registry& reg, float dt, const glm::vec3& gravity) {
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        if (rb.isStatic() || rb.sleeping) continue;

        // 0. 速度反算基准: 积分前的位置/朝向 (供 updateRigidBodyVelocities 算 v/omega)
        rb.x_prev = rb.x;
        rb.q_prev = rb.q;

        // 1. 重算世界反惯性张量 (q 可能上一帧被接触修正过)
        glm::mat3 R(rb.q);
        rb.inv_inertia_world = R * rb.inv_inertia_local * glm::transpose(R);

        // 2. 线速度: v += g*dt
        rb.v += gravity * dt;

        // 3. 位置: x += v*dt
        rb.x += rb.v * dt;

        // 4. 朝向: q += 0.5 * quat(0,omega) * q * dt
        glm::quat wq(0.0f, rb.omega.x, rb.omega.y, rb.omega.z);
        glm::quat dq = wq * rb.q;
        rb.q = rb.q + 0.5f * dt * dq;
        rb.q = glm::normalize(rb.q);
    }
}

} // namespace leo
