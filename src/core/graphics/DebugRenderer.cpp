#include "graphics/DebugRenderer.h"
#include "graphics/Vertex.h"
#include "scene/Scene.h"
#include "scene/Components.h"
#include "physics/PhysicsSystem.h"
#include "physics/Constraint.h"
#include "physics/Collider.h"
#include "physics/PhysicsTypes.h"
#include "Camera.h"
#include <glm/glm.hpp>
#include <vector>

namespace leo {

DebugRenderer::DebugRenderer() = default;
DebugRenderer::~DebugRenderer() = default;

bool DebugRenderer::init(Shader* debugShader) {
    m_shader = debugShader;
    // 初始空 mesh, 后续每帧 updateVertices
    // 点云: 预留容量上限, dynamic
    std::vector<Vertex> emptyPts(1024);
    std::vector<GLuint> emptyIdx;
    m_pointMesh.upload(emptyPts, emptyIdx, GL_POINTS, true);

    std::vector<Vertex> emptyLines(4096);
    m_lineMesh.upload(emptyLines, emptyIdx, GL_LINES, true);
    return true;
}

void DebugRenderer::shutdown() {
    m_pointMesh.release();
    m_lineMesh.release();
    m_shader = nullptr;
}

void DebugRenderer::render(Scene& scene, Camera& camera, float aspect,
                           bool drawPoints, bool drawColliders, bool drawConstraints) {
    if (!m_shader) return;
    auto& reg = scene.registry();

    m_shader->use();
    m_shader->setMat4("uView", camera.viewMatrix());
    m_shader->setMat4("uProj", camera.projectionMatrix(aspect));

    glm::mat4 identity(1.0f);
    m_shader->setMat4("uModel", identity);

    // debug 线框/点云: 关闭深度测试, 保证线框不被实心表面 (地面/台阶) 遮挡.
    // 否则落在地面上的箱子 AABB 线框会与地面共面, 被深度测试剔除 (z-fighting/不可见).
    GLboolean depthWasEnabled = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);

    // 1. 点云点 (绿色)
    if (drawPoints) {
        std::vector<Vertex> pts;
        auto view = reg.view<VerletPoint>();
        for (auto e : view) {
            VerletPoint& p = view.get<VerletPoint>(e);
            pts.push_back({p.x_current, {0,0,0}});
        }
        if (!pts.empty() && pts.size() <= 1024) {
            m_pointMesh.updateVertices(pts);
            m_shader->setVec3("uColor", glm::vec3(0.2f, 1.0f, 0.3f));
            // 点云点放大到 8px, 否则默认 1px 在 1280x720 下几乎不可见且采样不到
            glPointSize(8.0f);
            m_pointMesh.draw((GLsizei)pts.size());
        }
    }

    // 2. Collider AABB 线框 (青色) - 12 条边
    if (drawColliders) {
        std::vector<Vertex> lines;
        auto view = reg.view<ColliderRef>();
        for (auto e : view) {
            ColliderRef& cr = view.get<ColliderRef>(e);
            if (!cr.collider) continue;
            AABB b = cr.collider->getAABB();
            glm::vec3 p000{b.lo.x, b.lo.y, b.lo.z}, p100{b.hi.x, b.lo.y, b.lo.z};
            glm::vec3 p110{b.hi.x, b.hi.y, b.lo.z}, p010{b.lo.x, b.hi.y, b.lo.z};
            glm::vec3 p001{b.lo.x, b.lo.y, b.hi.z}, p101{b.hi.x, b.lo.y, b.hi.z};
            glm::vec3 p111{b.hi.x, b.hi.y, b.hi.z}, p011{b.lo.x, b.hi.y, b.hi.z};
            auto edge = [&](glm::vec3 a, glm::vec3 c) {
                lines.push_back({a, {0,0,0}});
                lines.push_back({c, {0,0,0}});
            };
            edge(p000,p100); edge(p100,p110); edge(p110,p010); edge(p010,p000);
            edge(p001,p101); edge(p101,p111); edge(p111,p011); edge(p011,p001);
            edge(p000,p001); edge(p100,p101); edge(p110,p111); edge(p010,p011);
        }
        if (!lines.empty() && lines.size() <= 4096) {
            m_lineMesh.updateVertices(lines);
            m_shader->setVec3("uColor", glm::vec3(0.3f, 0.9f, 1.0f));
            m_lineMesh.draw((GLsizei)lines.size());
        }
    }

    // 3. 距离约束连线 (黄色) - 从 PhysicsSystem 取
    if (drawConstraints) {
        std::vector<Vertex> lines;
        // 遍历场景里的 PhysicsSystem (通过 Application 注入会更干净, 这里从 Scene systems 取)
        // 简化: 直接遍历 entity 上的约束信息不易, 这里改为遍历 pair<VerletPoint> 无可行
        // 实际: PhysicsSystem 持有 constraints, DebugRenderer 需要引用它
        // 此处通过 Scene 的 systems 找 PhysicsSystem
        for (auto& sys : scene.systems()) {
            PhysicsSystem* phys = dynamic_cast<PhysicsSystem*>(sys.get());
            if (!phys) continue;
            for (auto& c : phys->constraints()) {
                auto* dc = dynamic_cast<DistanceConstraint*>(c.get());
                if (!dc) continue;
                auto ea = dc->entityA(), eb = dc->entityB();
                if (!reg.valid(ea) || !reg.valid(eb)) continue;
                auto* pa = reg.try_get<VerletPoint>(ea);
                auto* pb = reg.try_get<VerletPoint>(eb);
                if (!pa || !pb) continue;
                lines.push_back({pa->x_current, {0,0,0}});
                lines.push_back({pb->x_current, {0,0,0}});
            }
        }
        if (!lines.empty() && lines.size() <= 4096) {
            m_lineMesh.updateVertices(lines);
            m_shader->setVec3("uColor", glm::vec3(1.0f, 0.9f, 0.2f));
            m_lineMesh.draw((GLsizei)lines.size());
        }
    }

    // 恢复深度测试状态 (不影响后续帧的实心几何渲染)
    if (depthWasEnabled) glEnable(GL_DEPTH_TEST);
}

} // namespace leo
