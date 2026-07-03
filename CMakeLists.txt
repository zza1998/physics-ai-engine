cmake_minimum_required(VERSION 3.25)
project(leo_physics_engine
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # /utf-8 防中文注释 C4819 警告; /permissive- 严格标准; /W3 合理警告级别
    add_compile_options(/W3 /permissive- /utf-8)
endif()

# 统一可执行输出目录, 方便 POST_BUILD 拷贝 assets
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(third_party)
add_subdirectory(src)
