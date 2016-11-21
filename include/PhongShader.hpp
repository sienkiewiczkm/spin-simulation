#pragma once

#include "Effect.hpp"

#include <glm/glm.hpp>

#include <memory>

class PhongShader :
    public EffectBase
{
public:
    PhongShader();
    virtual ~PhongShader();

    void create();
    virtual void destroy() override;

    virtual void begin() override;
    virtual void end() override;

    void setLightDirection(glm::vec3 lightDirection);

    void setDiffuseTextureColor(glm::vec4 diffuseMultipler);
    void setDiffuseTexture(GLuint textureId);

    void setSolidColor(glm::vec3 color);
    void setSolidColor(glm::vec4 color);

private:
    void createShaders();

    GLuint _textureLocation;
    GLuint _lightDirectionLocation;
    GLuint _solidColorLocation;
    GLuint _diffuseColorLocation;

    GLuint _diffuseMap;
    glm::vec4 _diffuseMapColor;
    glm::vec4 _solidColor;
    glm::vec3 _lightDirection;
};
