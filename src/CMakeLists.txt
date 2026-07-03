# ---- leo_core 静态库 ----
add_library(leo_core STATIC
    core/Application.cpp
    core/Window.cpp
    core/Input.cpp
    core/Camera.cpp
    core/graphics/Renderer.cpp
    core/graphics/Shader.cpp
    core/graphics/Mesh.cpp
    core/graphics/MeshPrimitives.cpp
    core/scene/Scene.cpp
)
target_include_directories(leo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}        # src/   -> "core/xxx"
    ${CMAKE_CURRENT_SOURCE_DIR}/core   # src/core -> "graphics/xxx" "scene/xxx" "Application.h"
)
target_link_libraries(leo_core PUBLIC
    glfw
    glad
    glm::glm
    EnTT::EnTT
)
# M2: target_link_libraries(leo_core PUBLIC leo_physics)

# ---- leo_app 可执行 ----
add_executable(leo_app main.cpp)
target_link_libraries(leo_app PRIVATE leo_core)
set_target_properties(leo_app PROPERTIES OUTPUT_NAME "leo_physics")

# 运行时能找到 shader: 拷贝 assets/ 到可执行目录
add_custom_command(TARGET leo_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:leo_app>/assets
)

