#pragma once

#include <cassert>
#include <vector>

namespace spin
{

template <typename TPrecision>
class RungeKuttaMethod
{
public:
    RungeKuttaMethod(int dimension);
    virtual ~RungeKuttaMethod() = default;

protected:
    void setDimension(int dimension);

    void step(
        const TPrecision& t,
        const TPrecision& step,
        const std::vector<TPrecision>& input,
        std::vector<TPrecision>& output
    );

    virtual void evaluateFunction(
        const TPrecision& time,
        const std::vector<TPrecision>& unknownFunctionValue,
        std::vector<TPrecision>& output
    ) = 0;

private:
    std::vector<TPrecision> _output, _k1, _k2, _k3, _k4;

    std::vector<TPrecision> addVectors(
        const std::vector<TPrecision> &lhs,
        const std::vector<TPrecision> &rhs,
        TPrecision rhsFactor
    );

    int _dimension;
};

template <typename TPrecision>
RungeKuttaMethod<TPrecision>::RungeKuttaMethod(int dimension):
    _dimension{dimension}
{
}

template <typename TPrecision>
void RungeKuttaMethod<TPrecision>::setDimension(int dimension)
{
    _dimension = dimension;
}

template <typename TPrecision>
void RungeKuttaMethod<TPrecision>::step(
    const TPrecision& t,
    const TPrecision& step,
    const std::vector<TPrecision>& input,
    std::vector<TPrecision>& output
)
{
    std::vector<TPrecision> k1, k2, k3, k4;
    auto halfstep = step / 2;

    evaluateFunction(t, input, k1);
    evaluateFunction(t + halfstep, addVectors(input, k1, halfstep), k2);
    evaluateFunction(t + halfstep, addVectors(input, k2, halfstep), k3);
    evaluateFunction(t + step, addVectors(input, k3, step), k4);

    output.resize(_dimension);

    for (auto i = 0; i < _dimension; ++i)
    {
        output[i] = input[i] + (step/6) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
}

template <typename TPrecision>
std::vector<TPrecision> RungeKuttaMethod<TPrecision>::addVectors(
    const std::vector<TPrecision> &lhs,
    const std::vector<TPrecision> &rhs,
    TPrecision rhsFactor
)
{
    assert(lhs.size() == rhs.size());
    assert(lhs.size() == _dimension);

    std::vector<TPrecision> output(_dimension);
    for (auto i = 0; i < _dimension; ++i)
    {
        output[i] = lhs[i] + rhs[i] * rhsFactor;
    }
    return output;
}

}
