#include "SpinApplication.hpp"

#include <iostream>

#include "glm/gtc/matrix_transform.hpp"
#include "imgui.h"

#include "Config.hpp"
#include "DebugShapes.hpp"
#include "TextureUtils.hpp"

namespace spin
{

SpinApplication::SpinApplication():
    _enableCameraRotations{false},
    _cameraRotationSensitivity{0.2, 0.2}
{
}

SpinApplication::~SpinApplication()
{
}

void SpinApplication::onCreate()
{
    ImGuiApplication::onCreate();

    _phongEffect = std::make_shared<TexturedPhongEffect>();
    _phongEffect->create();

    _cube = createBox({1.0, 1.0, 1.0});
    _grid = std::make_shared<fw::Grid>(
        glm::ivec2{32, 32},
        glm::vec2{0.5f, 0.5f}
    );

    std::string resourcePath = RESOURCE("checker-base.png");
    std::cerr << "res path: " << resourcePath << std::endl;
    _testTexture = loadTextureFromFile(RESOURCE("textures/checker-base.png"));

    updateProjectionMatrix();
}

void SpinApplication::onDestroy()
{
    ImGuiApplication::onDestroy();
}

void SpinApplication::onUpdate(
    const std::chrono::high_resolution_clock::duration& deltaTime
)
{
    ImGuiApplication::onUpdate(deltaTime);

    if (ImGui::Begin("Example window"))
    {
        ImGui::Text("This is an example window");
    }
    ImGui::End();
}

void SpinApplication::onRender()
{
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    _phongEffect->setProjectionMatrix(_projectionMatrix);
    _phongEffect->setViewMatrix(_camera.getViewMatrix());
    _phongEffect->setModelMatrix({});
    _phongEffect->setTexture(_testTexture);
    _phongEffect->begin();
    _cube->render();
    _grid->render();
    _phongEffect->end();

    ImGuiApplication::onRender();
}

bool SpinApplication::onMouseButton(int button, int action, int mods)
{
    if (ImGuiApplication::onMouseButton(button, action, mods)) { return true; }

    if (GLFW_MOUSE_BUTTON_LEFT == button)
    {
        _enableCameraRotations = GLFW_PRESS == action;
    }

    return true;
}

bool SpinApplication::onMouseMove(glm::dvec2 newPosition)
{
    if (ImGuiApplication::onMouseMove(newPosition)) { return true; }

    if (_enableCameraRotations)
    {
        auto movement = getMouseMovement() * _cameraRotationSensitivity;
        _camera.rotate(movement.y, movement.x);
    }

    return true;
}

bool SpinApplication::onScroll(double xoffset, double yoffset)
{
    if (fw::ImGuiApplication::onScroll(xoffset, yoffset))
        return true;

    const double cMinimumDistance = 1.0;
    const double cMaximumDistance = 10.0;
    const double cZoomStep = 0.5;
    auto currentDistance = _camera.getDist();
    _camera.setDist(
        std::min(
            std::max(currentDistance + cZoomStep * yoffset, cMinimumDistance),
            cMaximumDistance
        )
    );

    return true;
}

bool SpinApplication::onResize()
{
    updateProjectionMatrix();
    return true;
}

void SpinApplication::updateProjectionMatrix()
{
    auto windowSize = getWindowSize();
    auto aspectRatio = static_cast<float>(windowSize.x) / windowSize.y;
    _projectionMatrix = glm::perspective(45.0f, aspectRatio, 0.5f, 100.0f);
}

}
