#include "Shapes.hpp"
#include "Common.hpp"
#include "glm/glm.hpp"

std::shared_ptr<Mesh<VertexNormalTexCoords>> createCone(
    float height,
    float radius,
    int circleSubdivisions,
    int circleCuts
)
{
    std::vector<VertexNormalTexCoords> vertices;
    std::vector<GLuint> indices;

    for (int angleStep = 0; angleStep <= circleSubdivisions; ++angleStep)
    {
        float progress = angleStep/static_cast<float>(circleSubdivisions);
        float angle = static_cast<float>(2.0*common::pi()*progress);

        float fx = sinf(angle);
        float fz = cosf(angle);

        for (int y = 0; y < circleCuts; ++y)
        {
            float heightProgress = y/static_cast<float>(circleCuts - 1);
            float fy = height * heightProgress;
            float currentRadius = radius * (1.0f - heightProgress);

            glm::vec3 position{currentRadius * fx, fy, currentRadius * fz};

            glm::vec3 flat{currentRadius * fx, 0.0f, currentRadius * fz};
            if (currentRadius > 10e-6)
            {
                flat = glm::normalize(flat);
            }

            glm::vec3 normal = glm::normalize(glm::vec3{
                flat.x * height / radius,
                radius / height,
                flat.z * height / radius
            });

            glm::vec2 texCoord{progress, heightProgress};

            vertices.push_back(VertexNormalTexCoords(
                position,
                normal,
                texCoord
            ));
        }
    }

    auto capIndex = vertices.size();
    vertices.push_back(VertexNormalTexCoords(
        glm::vec3{},
        glm::vec3{0, -1.0f, 0},
        glm::vec2{0.5f, 0.5f}
    ));

    for (int angleStep = 0; angleStep <= circleSubdivisions; ++angleStep)
    {
        float progress = angleStep/static_cast<float>(circleSubdivisions);
        float angle = static_cast<float>(2.0*common::pi()*progress);

        float fx = sinf(angle);
        float fz = cosf(angle);

        vertices.push_back(VertexNormalTexCoords(
            glm::vec3{radius * fx, 0, radius * fz},
            glm::vec3{0, -1.0f, 0.0f},
            glm::vec2{fx * 0.5f + 0.5f, fz * 0.5f + 0.5f}
        ));
    }

    for (int angleStep = 0; angleStep < circleSubdivisions; ++angleStep)
    {
        auto baseIndex = circleCuts * angleStep;
        for (int y = 0; y < circleCuts - 1; ++y)
        {
            indices.push_back(baseIndex + y);
            indices.push_back(baseIndex + y + 1);
            indices.push_back(baseIndex + y + circleCuts);

            indices.push_back(baseIndex + y + 1);
            indices.push_back(baseIndex + y + circleCuts);
            indices.push_back(baseIndex + y + circleCuts + 1);
        }
    }

    for (int angleStep = 0; angleStep < circleSubdivisions; ++angleStep)
    {
        indices.push_back(capIndex);
        indices.push_back(capIndex + angleStep + 1);
        indices.push_back(capIndex + angleStep + 2);
    }

    return std::make_shared<Mesh<VertexNormalTexCoords>>(vertices, indices);
}
