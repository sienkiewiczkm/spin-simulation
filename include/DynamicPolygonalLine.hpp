#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "Vertices.hpp"

namespace spin
{

class DynamicPolygonalLine
{
public:
    DynamicPolygonalLine(int numMaxVertices);
    ~DynamicPolygonalLine();

    void setNumMaxVertices(int numMaxVertices);
    int getNumMaxVertices() const;

    void setVertices(const std::vector<fw::VertexColor>& vertices);
    void render() const;

protected:
    GLuint _vao, _vbo;
    int _numVertices;
    int _numMaxVertices;

    void createBuffer();
    void destroyBuffer();
    void updateBuffer(const std::vector<fw::VertexColor> &vertices);
};

}
