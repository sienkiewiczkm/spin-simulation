#include "SpinApplication.hpp"

#include <cmath>
#include <iostream>

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "imgui.h"

#include "Config.hpp"
#include "Common.hpp"
#include "Shapes.hpp"
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
    _gravitationPlaneRenderingEnabled{true},
    _testDiagonalRotationX{0.0},
    _testDiagonalRotationY{0.0}
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
    _cone = createCone(1.0, 1.0);
    _cylinder = std::make_shared<Mesh<VertexNormalTexCoords>>(
        createCylinder(1.0, 1.0, 16)
    );

    _grid = std::make_shared<fw::Grid>(
        glm::ivec2{32, 32},
        glm::vec2{0.5f, 0.5f}
    );

    _testTexture = loadTextureFromFile(RESOURCE("textures/glass-tex.jpg"));

    updateProjectionMatrix();

    _cubeInitialQuaternion = glm::angleAxis(
        glm::radians(-90.0),
        glm::normalize(glm::dvec3{1.0, 0.0, 0.0})
    );

    _cubeInitialRotation = glm::mat4_cast(_cubeInitialQuaternion);

    _eulerEquations.setPreviousQuaternion(_cubeInitialQuaternion);
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

    if (_simulationEnabled) { updateRigidBody(deltaTime); }
}

void SpinApplication::onRender()
{
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    renderFrame();
    renderGroundGrid();
    if (_cubeRenderingEnabled) { renderCube(); }
    if (_cubeDiagonalRenderingEnabled) { renderCubeDiagonal(); }
    if (_trajectoryRenderingEnabled) { renderTrajectory(); }

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
        ImGui::DragFloat3(
            "Angular velocity",
            glm::value_ptr(_angularVelocity),
            0.001f
        );
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

void SpinApplication::drawArrow(
    glm::vec3 from,
    glm::vec3 to,
    glm::vec3 color,
    float thickness
)
{
    auto arrowThickness = 1.8f * thickness;
    auto arrowLength = glm::length(to - from);
    auto arrowHeadLength = std::min(2.0f * arrowThickness, arrowLength);

    glm::mat4 coneScale = glm::scale(
        glm::mat4{},
        {arrowThickness, arrowHeadLength, arrowThickness}
    );

    glm::mat4 cylinderScale = glm::scale(
        glm::mat4{},
        {thickness, arrowLength - arrowHeadLength, thickness}
    );

    auto rotationQuat = glm::quat(
        glm::vec3{0, 1.0f, 0.0f},
        glm::normalize(to - from)
    );

    auto rotationMat = glm::mat4_cast(rotationQuat);

    auto coneTranslation = glm::translate(
        glm::mat4{},
        from + (to - from) * ((arrowLength - arrowHeadLength)/(arrowLength))
    );

    auto cylinderTranslation = glm::translate(
        glm::mat4{},
        from + (to - from) * (arrowLength - arrowHeadLength)
            / (2.0f * arrowLength)
    );

    glm::mat4 coneTransformation = coneTranslation * rotationMat * coneScale;
    glm::mat4 cylinderTransformation = cylinderTranslation * rotationMat
        * cylinderScale;

    _phongEffect->setProjectionMatrix(_projectionMatrix);
    _phongEffect->setViewMatrix(_camera.getViewMatrix());
    _phongEffect->setDiffuseTextureColor({0.0f, 0.0f, 0.0f, 0.0f});
    _phongEffect->setSolidColor(glm::vec4{color, 1.0f});

    _phongEffect->setModelMatrix(coneTransformation);
    _phongEffect->begin();
    _cone->render();
    _phongEffect->end();

    _phongEffect->setModelMatrix(cylinderTransformation);
    _phongEffect->begin();
    _cylinder->render();
    _phongEffect->end();
}

glm::dmat3 SpinApplication::calculateBoxInertiaTensor()
{
    return calculateDiagonalInertiaTensor();
}

glm::dmat3 SpinApplication::calculateInertiaTensorOfPointMass(
    glm::dvec3 position,
    double mass
)
{
    double xSq = position.x * position.x;
    double ySq = position.y * position.y;
    double zSq = position.z * position.z;

    double xy = position.x * position.y;
    double xz = position.x * position.z;
    double yz = position.y * position.z;

    return mass * glm::dmat3{
        { ySq + zSq, -xy, -xz},
        { -xy, xSq + zSq, -yz},
        { -xz, -yz, ySq + xSq}
    };
}

glm::dmat3 SpinApplication::calculateEigenvectorMatrix() const
{
    return glm::dmat3{
        {-1.0, 0.0, 1.0},
        {-1.0, 1.0, 0.0},
        { 1.0, 1.0, 1.0}
    };
}

glm::dmat3 SpinApplication::calculateDiagonalInertiaTensor() const
{
    auto mass = _cubeSize * _cubeSize * _cubeSize * _cubeDensity;
    auto lambda1 = 11.0 * _cubeSize * _cubeSize * mass / 12.0;
    auto lambda2 = _cubeSize * _cubeSize * mass / 6.0;

    return glm::dmat3{
        {lambda1, 0.0, 0.0},
        {0.0, lambda1, 0.0},
        {0.0, 0.0, lambda2}
    };
}

float SpinApplication::getGravity() const
{
    return _gravityEnabled ? _gravityConstant : 0.0f;
}

void SpinApplication::updateRigidBody(
    const std::chrono::high_resolution_clock::duration& deltaTime
)
{
    auto deltaSeconds = std::chrono::duration<double>(deltaTime);
    _eulerEquations.setInertiaTensor(calculateBoxInertiaTensor());

    _testDiagonalRotationX += 0.3 * deltaSeconds.count();
    _testDiagonalRotationY += 0.1 * deltaSeconds.count();

    auto boxMass = _cubeDensity * _cubeSize * _cubeSize * _cubeSize;
    _eulerEquations.setPreviousAngularVelocity(_angularVelocity);

    glm::dvec3 force{0, -boxMass*getGravity(), 0};
    auto eigenvectorMatrix = calculateEigenvectorMatrix();
    auto invEigenvectorMatrix = glm::inverse(eigenvectorMatrix);
    _bodyForcePoint = invEigenvectorMatrix * glm::dvec3{
        _cubeSize / 2,
        _cubeSize / 2,
        _cubeSize / 2
    };

    _eulerEquations.setBodyForcePoint(_bodyForcePoint);
    _eulerEquations.setTorque(force);

    _eulerEquations.update(deltaSeconds.count());
    _angularVelocity = _eulerEquations.getAngularVelocity();
    auto quat = _eulerEquations.getQuaternion();
    _cubeInitialRotation = glm::mat4_cast(quat);
}

void SpinApplication::renderGroundGrid()
{
    _phongEffect->setModelMatrix({});
    _phongEffect->setDiffuseTextureColor({0.0f, 0.0f, 0.0f, 0.0f});
    _phongEffect->setSolidColor({1.0f, 0.0f, 0.0f, 1.0f});
    _phongEffect->begin();
    _grid->render();
    _phongEffect->end();
}

void SpinApplication::renderFrame()
{
    drawArrow({0, 0, 0}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.01f);
    drawArrow({0, 0, 0}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.01f);
    drawArrow({0, 0, 0}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, 0.01f);
}

void SpinApplication::renderCube()
{
    glm::mat4 cubeTransformation = _cubeInitialRotation;

    auto eigenvectorMatrix = calculateEigenvectorMatrix();
    auto invEigen = glm::inverse(eigenvectorMatrix);
    cubeTransformation *= glm::mat4{
        { invEigen[0], 0 },
        { invEigen[1], 0 },
        { invEigen[2], 0 },
        { 0, 0, 0, 1.0 }
    };

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
    _phongEffect->setSolidColor({0.0f, 0.0f, 0.0f, 0.7f});
    _phongEffect->begin();
    _cube->render();
    _phongEffect->end();

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
}

void SpinApplication::renderCubeDiagonal()
{
    double diagonalLength = _cubeSize * sqrt(3.0);

    // todo: transform cube diagonal using real matrices
    auto rotation = glm::rotate(
        glm::dmat4{},
        _testDiagonalRotationY,
        {0, 1, 0}
    );

    rotation = glm::rotate(rotation, _testDiagonalRotationX, {1, 0, 0});
    glm::dvec4 objectSpaceDiagonal{0, diagonalLength, 0, 1.0};

    glm::vec3 worldSpaceDiagonal{glm::dvec3{rotation * objectSpaceDiagonal}};
    drawArrow({0, 0, 0}, worldSpaceDiagonal, {1.0f, 0.0f, 1.0f}, 0.02f);
}

void SpinApplication::renderTrajectory()
{
}

void SpinApplication::renderGravityVector()
{
}

}
