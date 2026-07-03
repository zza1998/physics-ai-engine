#pragma once

#include "graphics/Mesh.h"

namespace leo {

// 1x1x1 立方体, 居中, 每面独立顶点 (保证每面法线正确)
Mesh makeCube();

// 平面: y=0, 法线 +Y, 给定宽(width, x方向)和深(depth, z方向)
Mesh makePlane(float width, float depth);

} // namespace leo
