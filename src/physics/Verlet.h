#pragma once

#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "physics/PhysicsTypes.h"

namespace leo {

// Verlet 积分: v = (x_current - x_prev) * damping; x_prev = x_current; x_current += v + a*dt²
// 遍历所有 VerletPoint, 静态点 (inv_mass==0) 跳过
void integrate(entt::registry& reg, float dt, const glm::vec3& gravity, float damping);

} // namespace leo
