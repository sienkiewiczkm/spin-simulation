#pragma once

#include <memory>
#include <vector>

#include "Mesh.hpp"
#include "Vertices.hpp"
#include "OpenGLHeaders.hpp"

std::shared_ptr<Mesh<VertexNormalTexCoords>> createCone(
    float height,
    float radius,
    int circleSubdivisions = 16,
    int circleCuts = 8
);
