#pragma once

#include <entt/entt.hpp>
#include <glm/glm.hpp>

namespace leo {

// 半隐式 Euler 积分 RigidBody:
//   inv_inertia_world = R * inv_inertia_local * R^T  (R = mat3(q))
//   v += gravity * dt
//   x += v * dt
//   q += 0.5 * quat(0, omega) * q * dt;  q = normalize(q)
// 静态体 (inv_mass==0) 跳过. 积分后存 x_prev/q_prev 供接触求解速度反算用.
void integrateRigidBodies(entt::registry& reg, float dt, const glm::vec3& gravity);

} // namespace leo
