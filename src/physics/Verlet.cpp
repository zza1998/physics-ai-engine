#include "physics/Verlet.h"

namespace leo {

void integrate(entt::registry& reg, float dt, const glm::vec3& gravity, float damping) {
    auto view = reg.view<VerletPoint>();
    for (auto e : view) {
        VerletPoint& p = view.get<VerletPoint>(e);
        if (p.isStatic()) continue;

        glm::vec3 v = (p.x_current - p.x_prev) * damping;
        p.x_prev = p.x_current;
        p.x_current = p.x_current + v + gravity * (dt * dt);
    }
}

} // namespace leo
