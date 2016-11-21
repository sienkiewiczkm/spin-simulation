#include "SpinApplication.hpp"

#include <cmath>
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
    _cameraRotationSensitivity{0.2, 0.2},
    _cubeSize{1.0f},
    _cubeDensity{1.0f},
    _gravityConstant{9.807f},
    _simulationEnabled{false},
    _gravityEnabled{false},
    _trajectoryLength{500},
    _cubeRenderingEnabled{true},
    _cubeDiagonalRenderingEnabled{true},
    _trajectoryRenderingEnabled{true},
    _gravitationVectorRenderingEnabled{true},
    _gravitationPlaneRenderingEnabled{true}
{
}

SpinApplication::~SpinApplication()
{
}

void SpinApplication::onCreate()
{
    ImGuiApplication::onCreate();

    _phongEffect = std::make_shared<PhongShader>();
    _phongEffect->create();

    _cube = createBox({1.0, 1.0, 1.0});
    _grid = std::make_shared<fw::Grid>(
        glm::ivec2{32, 32},
        glm::vec2{0.5f, 0.5f}
    );

    _testTexture = loadTextureFromFile(RESOURCE("textures/glass-tex.jpg"));

    updateProjectionMatrix();

    _cubeInitialRotation = glm::rotate(
        glm::mat4{},
        glm::radians(35.254f),
        glm::vec3{0.0f, 0.0f, 1.0f}
    );

    _cubeInitialRotation = glm::rotate(
        _cubeInitialRotation,
        glm::radians(-45.0f),
        glm::vec3{1.0f, 0.0f, 0.0f}
    );
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
    showBoxSettings();
    updateGravityChart();
}

void SpinApplication::onRender()
{
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    _phongEffect->setModelMatrix({});
    _phongEffect->setDiffuseTextureColor({0.0f, 0.0f, 0.0f, 0.0f});
    _phongEffect->setSolidColor({1.0f, 0.0f, 0.0f, 1.0f});
    _phongEffect->begin();
    _grid->render();
    _phongEffect->end();

    if (_cubeRenderingEnabled)
    {
        glm::mat4 cubeTransformation = glm::rotate(
            glm::mat4{},
            glm::radians(_zRotationDegrees),
            glm::vec3{0.0f, 0.0f, 1.0f}
        );

        cubeTransformation *= _cubeInitialRotation;

        cubeTransformation = glm::translate(
            cubeTransformation,
            glm::vec3{_cubeSize / 2.0, _cubeSize / 2.0, _cubeSize / 2.0}
        );

        cubeTransformation = glm::scale(
            cubeTransformation,
            glm::vec3{_cubeSize, _cubeSize, _cubeSize}
        );

        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        _phongEffect->setProjectionMatrix(_projectionMatrix);
        _phongEffect->setViewMatrix(_camera.getViewMatrix());
        _phongEffect->setDiffuseTextureColor({1.0f, 1.0f, 1.0f, 1.0f});
        _phongEffect->setDiffuseTexture(_testTexture);
        _phongEffect->setModelMatrix(cubeTransformation);
        _phongEffect->setSolidColor({0.0f, 0.0f, 0.0f, 0.5f});
        _phongEffect->begin();
        _cube->render();
        _phongEffect->end();

        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
    }


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

void SpinApplication::showBoxSettings()
{
    ImGui::ShowTestWindow();
    if (!ImGui::Begin("Simulation settings"))
    {
        ImGui::End();
        return;
    }

    ImGui::Checkbox("Enable simulation", &_simulationEnabled);

    if (ImGui::CollapsingHeader("Simulation parameters"))
    {
        ImGui::DragFloat("Cube size", &_cubeSize, 0.005f);
        ImGui::DragFloat("Cube density", &_cubeDensity, 0.001f);
        ImGui::DragFloat("Cube rotation (deg)", &_zRotationDegrees, 0.5f);
        ImGui::DragFloat("Angular velocity", &_angularVelocity, 0.001f);
    }

    if (ImGui::CollapsingHeader("Gravity"))
    {
        ImGui::Checkbox("Enable gravity", &_gravityEnabled);
        ImGui::SliderFloat("Gravity", &_gravityConstant, 0.0f, 50.0f);
        ImGui::PlotLines(
            "Gravity history",
            _gravityHistory.data(),
            _gravityHistory.size(),
            0,
            nullptr,
            0.0f,
            51.0f,
            ImVec2(0, 80)
        );
    }

    if (ImGui::CollapsingHeader("Visuals"))
    {
        ImGui::Checkbox("Display cube", &_cubeRenderingEnabled);
        ImGui::Checkbox("Display diagonal", &_cubeDiagonalRenderingEnabled);
        ImGui::Checkbox("Display trajectory", &_trajectoryRenderingEnabled);
        ImGui::SliderInt("Trajectory length", &_trajectoryLength, 0, 1000);
        ImGui::Checkbox(
            "Display gravity vector",
            &_gravitationVectorRenderingEnabled
        );
        ImGui::Checkbox(
            "Display gravity plane",
            &_gravitationPlaneRenderingEnabled
        );
    }

    ImGui::End();
}

void SpinApplication::updateGravityChart()
{
    _gravityHistory.push_back(getGravity());

    if (_gravityHistory.size() > cMaximumGravityHistorySize)
    {
        _gravityHistory.erase(std::begin(_gravityHistory));
    }
}

float SpinApplication::getGravity() const
{
    return _gravityEnabled ? _gravityConstant : 0.0f;
}

}
