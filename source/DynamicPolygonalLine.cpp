#include "DynamicPolygonalLine.hpp"

namespace spin
{

DynamicPolygonalLine::DynamicPolygonalLine(int numMaxVertices):
    _numMaxVertices(numMaxVertices)
{
    createBuffer();
}

DynamicPolygonalLine::~DynamicPolygonalLine()
{
    destroyBuffer();
}

void DynamicPolygonalLine::setNumMaxVertices(int numMaxVertices)
{
    _numMaxVertices = numMaxVertices;
}

int DynamicPolygonalLine::getNumMaxVertices() const
{
    return _numMaxVertices;
}

void DynamicPolygonalLine::setVertices(
    const std::vector<fw::VertexColor>& vertices
)
{
    updateBuffer(vertices);
}

void DynamicPolygonalLine::render() const
{
    glBindVertexArray(_vao);
    glDrawArrays(GL_LINE_STRIP, 0, _numVertices);
    glBindVertexArray(0);
}

void DynamicPolygonalLine::createBuffer()
{
    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(fw::VertexColor) * _numMaxVertices,
        nullptr,
        GL_DYNAMIC_DRAW
    );

    fw::VertexColor::setupAttribPointers();

    glBindVertexArray(0);
}

void DynamicPolygonalLine::destroyBuffer()
{
    if (_vbo != 0) { glDeleteBuffers(1, &_vbo); }
    if (_vao != 0) { glDeleteVertexArrays(1, &_vao); }
    _vao = _vbo = 0;
}

void DynamicPolygonalLine::updateBuffer(
    const std::vector<fw::VertexColor> &vertices
)
{
    assert(vertices.size() < _numMaxVertices);

    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferSubData(
        GL_ARRAY_BUFFER,
        0,
        vertices.size() * sizeof(fw::VertexColor),
        vertices.data()
    );

    _numVertices = vertices.size();
}

}
