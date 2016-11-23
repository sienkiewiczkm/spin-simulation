#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "RungeKuttaMethod.hpp"

namespace spin
{

class EulerRotationEquations:
    public RungeKuttaMethod<double>
{
public:
    EulerRotationEquations();
    virtual ~EulerRotationEquations() = default;

    void setPreviousQuaternion(glm::dquat quaternion);
    void setPreviousAngularVelocity(glm::dvec3 angularVelocity);

    void setBodyForcePoint(glm::dvec3 bodyForcePoint);
    void setTorque(glm::dvec3 torque);
    void setInertiaTensor(glm::dmat3 inertiaTensor);

    void update(double dt);

    glm::dvec3 getAngularVelocity();
    glm::dquat getQuaternion();

protected:
    void packVector(
        std::vector<double>& output,
        const glm::dvec3& angularVelocityDerivative,
        const glm::dquat& quaternionDerivative
    );

    void unpackVector(
        const std::vector<double>& input,
        glm::dvec3& angularVelocity,
        glm::dquat& quaternion
    );

    virtual void evaluateFunction(
        const double& time,
        const std::vector<double>& unknownFunctionValue,
        std::vector<double>& output
    ) override;

private:
    glm::dvec3 _previousAngularVelocity;
    glm::dquat _previousQuaternion;
    glm::dvec3 _torque;
    glm::dvec3 _bodyForcePoint;
    glm::dmat3 _inertiaTensor, _inertiaTensorInv;
    glm::dvec3 _angularVelocity;
    glm::dquat _quaternion;
};

}
