#version 330 core
precision highp float;

out vec2 texPos;
uniform mat3x2 modelMat;
uniform mat3x2 textureMat;

const vec2 aPos[4] = vec2[](
    vec2(0., 0.),
    vec2(1., 0.),
    vec2(0., 1.),
    vec2(1., 1.));

void main() {
    vec3 pos = vec3(aPos[gl_VertexID], 1.);
    gl_Position = vec4((modelMat * pos) * 2 - 1, 0, 1);
    texPos = textureMat * pos;
}
