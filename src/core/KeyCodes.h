#pragma once

// 薄封装: 直接用 GLFW 键码常量值, 预留 M2 重映射空间
// 数值与 GLFW 一致, 可直接传 Input::isKeyDown(GLFW_KEY_xxx)

namespace leo {

// 不在此重复定义 GLFW 键码, 包含 <GLFW/glfw3.h> 后直接用 GLFW_KEY_*
// 这个头文件为未来抽象键码留位置

} // namespace leo
