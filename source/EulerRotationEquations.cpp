#include "EulerRotationEquations.hpp"

namespace spin
{

EulerRotationEquations::EulerRotationEquations():
    RungeKuttaMethod<double>(7)
{
}

void EulerRotationEquations::setPreviousQuaternion(glm::dquat quaternion)
{
    _previousQuaternion = quaternion;
    _quaternion = quaternion;
}

void EulerRotationEquations::setPreviousAngularVelocity(
    glm::dvec3 angularVelocity
)
{
    _previousAngularVelocity = angularVelocity;
    _angularVelocity = angularVelocity;
}

void EulerRotationEquations::setExternalForce(
    glm::dvec3 forcePoint,
    glm::dvec3 force
)
{
    _forcePoint = forcePoint;
    _force = force;
}

void EulerRotationEquations::setInertiaTensor(glm::dmat3 inertiaTensor)
{
    _inertiaTensor = inertiaTensor;
    _inertiaTensorInv = glm::inverse(inertiaTensor);
}

void EulerRotationEquations::update(double dt)
{
    dt = std::min(dt, 1.0/45);

    _previousAngularVelocity = _angularVelocity;
    _previousQuaternion = _quaternion;

    std::vector<double> input, output;
    packVector(input, _previousAngularVelocity, _previousQuaternion);
    step(0.0, dt, input, output);
    unpackVector(output, _angularVelocity, _quaternion);
    _quaternion = glm::normalize(_quaternion);
}

glm::dvec3 EulerRotationEquations::getAngularVelocity()
{
    return _angularVelocity;
}

glm::dquat EulerRotationEquations::getQuaternion()
{
    return _quaternion;
}

void EulerRotationEquations::packVector(
    std::vector<double>& output,
    const glm::dvec3& angularVelocityDerivative,
    const glm::dquat& quaternionDerivative
)
{
    output.resize(3 + 4);
    output[0] = angularVelocityDerivative.x;
    output[1] = angularVelocityDerivative.y;
    output[2] = angularVelocityDerivative.z;
    output[3] = quaternionDerivative.w;
    output[4] = quaternionDerivative.x;
    output[5] = quaternionDerivative.y;
    output[6] = quaternionDerivative.z;
}

void EulerRotationEquations::unpackVector(
    const std::vector<double>& input,
    glm::dvec3& angularVelocity,
    glm::dquat& quaternion
)
{
    assert(input.size() == (3+4));
    angularVelocity = glm::dvec3{input[0], input[1], input[2]};
    quaternion = glm::dquat{input[3], input[4], input[5], input[6]};
}

void EulerRotationEquations::evaluateFunction(
    const double& time,
    const std::vector<double>& unknownFunctionValue,
    std::vector<double>& output
)
{
    glm::dvec3 angularVelocity;
    glm::dquat quaternion;
    unpackVector(unknownFunctionValue, angularVelocity, quaternion);

    auto quaternionMat = glm::mat3_cast(glm::normalize(quaternion));
    auto torque = glm::cross(
        _forcePoint,
        glm::inverse(quaternionMat) * _force
    );

    auto IW = _inertiaTensor * angularVelocity;
    auto angularMomentumDerivative = _inertiaTensorInv *
        (torque + glm::cross(IW, angularVelocity));
    auto quaternionDerivative =
        (glm::normalize(quaternion) * glm::dquat{0, angularVelocity}) / 2.0;

    packVector(output, angularMomentumDerivative, quaternionDerivative);
}

}
