#include "PhongShader.hpp"
#include "Config.hpp"
#include <iostream>

#include <glm/gtc/type_ptr.hpp>

using namespace std;

PhongShader::PhongShader():
    _diffuseMapColor{0.0, 0.0, 0.0, 0.0},
    _solidColor{1.0, 0.0, 0.0, 1.0},
    _lightDirection{0.0, 1.0, 0.0},
    _diffuseMap{0}
{
}

PhongShader::~PhongShader()
{
}

void PhongShader::create()
{
    createShaders();

    _textureLocation = glGetUniformLocation(
        _shaderProgram->getId(),
        "TextureSlot1"
    );

    _lightDirectionLocation = glGetUniformLocation(
        _shaderProgram->getId(),
        "LightDirection"
    );

    _solidColorLocation = glGetUniformLocation(
        _shaderProgram->getId(),
        "SolidColor"
    );

    _diffuseColorLocation = glGetUniformLocation(
        _shaderProgram->getId(),
        "DiffuseMapColor"
    );
}

void PhongShader::destroy()
{
}

void PhongShader::begin()
{
    _shaderProgram->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _diffuseMap);
    glUniform1i(_textureLocation, 0);

    glUniform3fv(_lightDirectionLocation, 1, glm::value_ptr(_lightDirection));
    glUniform4fv(_solidColorLocation, 1, glm::value_ptr(_solidColor));
    glUniform4fv(_diffuseColorLocation, 1, glm::value_ptr(_diffuseMapColor));
}

void PhongShader::end()
{
}

void PhongShader::setLightDirection(glm::vec3 lightDirection)
{
    _lightDirection = lightDirection;
}

void PhongShader::setDiffuseTextureColor(glm::vec4 diffuseMultipler)
{
    _diffuseMapColor = diffuseMultipler;
}

void PhongShader::setDiffuseTexture(GLuint textureId)
{
    _diffuseMap = textureId;
}

void PhongShader::setSolidColor(glm::vec3 color)
{
    _solidColor = glm::vec4{color, 1.0};
}

void PhongShader::setSolidColor(glm::vec4 color)
{
    _solidColor = color;
}

void PhongShader::createShaders()
{
    shared_ptr<Shader> vs = make_shared<Shader>();
    std::cout << "looking for shader in directory: " << RESOURCE("shaders/PhongShader.vert") << std::endl;
    vs->addSourceFromFile(RESOURCE("shaders/PhongShader.vert"));
    vs->compile(GL_VERTEX_SHADER);

    shared_ptr<Shader> fs = make_shared<Shader>();
    fs->addSourceFromFile(RESOURCE("shaders/PhongShader.frag"));
    fs->compile(GL_FRAGMENT_SHADER);

    _shaderProgram = make_shared<ShaderProgram>();
    _shaderProgram->attach(vs.get());
    _shaderProgram->attach(fs.get());
    _shaderProgram->link();
}
