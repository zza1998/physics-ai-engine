#include <glad/gl.h>
#include "graphics/Renderer.h"
#include "scene/Scene.h"
#include "scene/Components.h"
#include "Camera.h"

namespace leo {

Renderer::Renderer() = default;

bool Renderer::init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
    return true;
}

void Renderer::setViewport(int x, int y, int w, int h) {
    glViewport(x, y, w, h);
}

void Renderer::draw(Shader& shader, const Mesh& mesh, const glm::mat4& model) {
    shader.setMat4("uModel", model);
    mesh.draw();
}

void Renderer::applyLight(Shader& shader, const Light& light, const glm::vec3& cameraPos) {
    shader.setVec3("uLightDir", light.direction);
    shader.setVec3("uLightColor", light.color * light.intensity);
    shader.setVec3("uViewPos", cameraPos);
}

void Renderer::renderScene(Scene& scene, Camera& camera, float aspect) {
    auto& reg = scene.registry();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 取第一个方向光 (M1 只支持一个)
    Light* dirLight = nullptr;
    auto lightView = reg.view<Light>();
    for (auto e : lightView) {
        Light& l = lightView.get<Light>(e);
        if (l.type == Light::Type::Directional) { dirLight = &l; break; }
    }

    // 渲染所有 Renderable + Transform 实体
    auto view = reg.view<Transform, Renderable>();
    for (auto e : view) {
        auto& tf = view.get<Transform>(e);
        auto& r  = view.get<Renderable>(e);
        if (!r.mesh || !r.shader) continue;

        Shader& shader = *r.shader;
        shader.use();
        shader.setMat4("uView", camera.viewMatrix());
        shader.setMat4("uProj", camera.projectionMatrix(aspect));
        shader.setVec3("uAlbedo", r.albedo);
        shader.setFloat("uShininess", r.shininess);

        if (dirLight) applyLight(shader, *dirLight, camera.position());

        draw(shader, *r.mesh, tf.modelMatrix());
    }
}

} // namespace leo
