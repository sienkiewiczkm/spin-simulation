#pragma once

#include "Grid.hpp"
#include "ImGuiApplication.hpp"
#include "Mesh.hpp"
#include "OrbitingCamera.hpp"
#include "TexturedPhongEffect.hpp"
#include "Vertices.hpp"

#include <memory>

#include "glm/glm.hpp"

namespace spin
{

class SpinApplication:
    public fw::ImGuiApplication
{
public:
    SpinApplication();
    virtual ~SpinApplication();

protected:
    virtual void onCreate() override;
    virtual void onDestroy() override;
    virtual void onUpdate(
        const std::chrono::high_resolution_clock::duration& deltaTime
    ) override;
    virtual void onRender() override;

    virtual bool onMouseButton(int button, int action, int mods) override;
    virtual bool onMouseMove(glm::dvec2 newPosition) override;
    virtual bool onScroll(double xoffset, double yoffset) override;
    virtual bool onResize() override;

    void updateProjectionMatrix();

private:
    std::shared_ptr<TexturedPhongEffect> _phongEffect;
    std::shared_ptr<Mesh<VertexNormalTexCoords>> _cube;

    std::shared_ptr<fw::Grid> _grid;
    OrbitingCamera _camera;
    glm::mat4 _projectionMatrix;
    bool _enableCameraRotations;

    glm::dvec2 _cameraRotationSensitivity;
    GLuint _testTexture;
};

}
