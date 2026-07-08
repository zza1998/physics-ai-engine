# ---- leo_core 静态库 (含物理源文件, 方案A 避免循环依赖) ----
add_library(leo_core STATIC
    core/Application.cpp
    core/Window.cpp
    core/Input.cpp
    core/Camera.cpp
    core/graphics/Renderer.cpp
    core/graphics/Shader.cpp
    core/graphics/Mesh.cpp
    core/graphics/MeshPrimitives.cpp
    core/graphics/DebugRenderer.cpp
    core/ui/ImGuiLayer.cpp
    core/scene/Scene.cpp
    physics/Verlet.cpp
    physics/Constraint.cpp
    physics/Collider.cpp
    physics/PhysicsSystem.cpp
    physics/RBIntegrate.cpp
)
target_include_directories(leo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}        # src/   -> "core/xxx" "physics/xxx"
    ${CMAKE_CURRENT_SOURCE_DIR}/core   # src/core -> "graphics/xxx" "scene/xxx"
)
target_link_libraries(leo_core PUBLIC
    glfw
    glad
    glm::glm
    EnTT::EnTT
    imgui_backends
)

# ---- imgui_backends: ImGui 核心 + GLFW/OpenGL3 backends ----
set(IMGUI_DIR ${FETCHCONTENT_BASE_DIR}/imgui-src CACHE PATH "imgui source dir")
add_library(imgui_backends STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_backends PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(imgui_backends PUBLIC glfw glad)
# imgui_impl_opengl3 用 glad2 加载 GL, 需定义此宏
target_compile_definitions(imgui_backends PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD2)

# ---- leo_app 可执行 ----
add_executable(leo_app main.cpp)
target_link_libraries(leo_app PRIVATE leo_core)
set_target_properties(leo_app PROPERTIES OUTPUT_NAME "leo_physics")

# 运行时能找到 shader: 拷贝 assets/ 到可执行目录
add_custom_command(TARGET leo_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:leo_app>/assets
)
