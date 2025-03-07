#version 330 core

in vec2 texPos;
uniform sampler2D _texture;

void main() {
    gl_FragColor = vec4(texture(_texture, texPos).rgb, 1);
}
