# ---- leo_core 静态库 ----
add_library(leo_core STATIC
    core/Application.cpp
    core/Window.cpp
    core/Input.cpp
    core/Camera.cpp
    core/Logger.cpp
    core/graphics/Renderer.cpp
    core/graphics/Shader.cpp
    core/graphics/Mesh.cpp
    core/graphics/MeshPrimitives.cpp
    core/scene/Scene.cpp
)
target_include_directories(leo_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(leo_core PUBLIC
    glfw
    glad::glad
    glm::glm
    EnTT::EnTT
)
# M2: target_link_libraries(leo_core PUBLIC leo_physics)

# ---- leo_app 可执行 ----
add_executable(leo_app main.cpp)
target_link_libraries(leo_app PRIVATE leo_core)
set_target_properties(leo_app PROPERTIES OUTPUT_NAME "leo_physics")
