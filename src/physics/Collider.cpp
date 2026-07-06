#include "physics/Collider.h"
#include <glm/glm.hpp>
#include <limits>

namespace leo {

PointCloudBox::PointCloudBox(entt::registry& reg, std::array<entt::entity, 8> points)
    : m_reg(reg), m_points(points) {}

AABB PointCloudBox::getAABB() const {
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (auto e : m_points) {
        auto* p = m_reg.try_get<VerletPoint>(e);
        if (!p) continue;
        lo = glm::min(lo, p->x_current);
        hi = glm::max(hi, p->x_current);
    }
    return AABB{lo, hi};
}

Contact PointCloudBox::collidePoint(const glm::vec3& p) const {
    return pointVsAABB(p, getAABB());
}

} // namespace leo
