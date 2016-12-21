#pragma once

#include "Grid.hpp"
#include "ImGuiApplication.hpp"
#include "Mesh.hpp"
#include "OrbitingCamera.hpp"
#include "PhongShader.hpp"
#include "Vertices.hpp"
#include "EulerRotationEquations.hpp"
#include "DynamicPolygonalLine.hpp"

#include <memory>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

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
    void showBoxSettings();
    void updateGravityChart();

    void drawArrow(
        glm::vec3 from,
        glm::vec3 to,
        glm::vec3 color,
        float thickness
    );

    glm::dmat3 calculateBoxInertiaTensor();
    glm::dmat3 calculateInertiaTensorOfPointMass(
        glm::dvec3 position,
        double mass
    );

private:
    void setupSimulation();

    void updateRigidBody(const double& deltaTime, bool addTrajectory);

    void renderGroundGrid();
    void renderFrame();
    void renderCube();
    void renderCubeDiagonal();
    void renderTrajectory();
    void renderGravityVector();
    void renderTorqueVector();

    void addTrajectoryPoint(glm::vec3 trajectoryPoint);

    double getGravity() const;

    const int cMaximumGravityHistorySize = 200;

    std::shared_ptr<PhongShader> _phongEffect;
    std::shared_ptr<Mesh<VertexNormalTexCoords>> _cube;

    std::shared_ptr<Mesh<VertexNormalTexCoords>> _cone;
    std::shared_ptr<Mesh<VertexNormalTexCoords>> _cylinder;

    float _cubeSize;
    float _cubeDensity;
    glm::dquat _cubeVisualAdjustmentQuaternion;
    glm::dquat _cubeQuaternion;

    glm::vec3 _angularVelocity;
    glm::dvec3 _currentAngularVelocity;

    float _diagonalAngularVelocity;
    float _diagonalTiltAngle;

    bool _showDangerousSimulationSettings;
    bool _cubeRenderingEnabled;
    bool _cubeDiagonalRenderingEnabled;
    bool _trajectoryRenderingEnabled;
    bool _gravitationVectorRenderingEnabled;
    bool _gravitationPlaneRenderingEnabled;
    bool _torqueRenderingEnabled;

    std::vector<glm::vec3> _trajectory;
    std::shared_ptr<DynamicPolygonalLine> _dynamicPolygonalLine;

    bool _simulationEnabled;
    bool _gravityEnabled;
    float _gravityConstant;
    std::vector<float> _gravityHistory;

    glm::dvec3 _bodyForcePoint;
    glm::dvec3 _forceVector;

    glm::dvec3 _tensorSpaceCenter;
    glm::dvec3 _tensorSpaceGravity;
    glm::dvec3 _tensorSpaceTorque;

    std::shared_ptr<fw::Grid> _grid;
    OrbitingCamera _camera;
    glm::mat4 _projectionMatrix;
    bool _enableCameraRotations;

    std::shared_ptr<EulerRotationEquations> _eulerEquations;

    glm::dvec2 _cameraRotationSensitivity;
    GLuint _testTexture;
};

}
