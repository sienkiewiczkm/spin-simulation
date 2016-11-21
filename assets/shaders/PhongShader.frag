#version 330 core

in vec3 Normal;
in vec2 TexCoord;
out vec4 color;

uniform sampler2D TextureSlot1;
uniform vec3 LightDirection;
uniform vec4 SolidColor;
uniform vec4 DiffuseMapColor;

void main(void)
{
    vec3 lightDirection = normalize(LightDirection);
    vec3 diffuseLightColor = vec3(1.0, 1.0, 1.0);

    float diffuse = dot(lightDirection, normalize(Normal));

    vec3 albedo = clamp(
        SolidColor.rgb
            + DiffuseMapColor.rgb * texture(TextureSlot1, TexCoord).rgb,
        0.0,
        1.0
    );

    vec3 ambientLight = vec3(0.2, 0.2, 0.2);
    vec3 ambientPart = albedo * ambientLight;
    vec3 diffusePart = albedo * diffuseLightColor * diffuse;

    color = vec4(ambientPart + diffusePart, SolidColor.a);
}

