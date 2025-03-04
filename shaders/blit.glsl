#version 330 core

in vec2 texPos;
uniform sampler2D _texture;

void main() {
    ivec2 cell = ivec2(gl_FragCoord.xy / 10);
    vec4 texColor = vec4(texture(_texture, texPos).rgb, 1);
    vec4 bgColor = vec4(vec3(
        (cell.x + cell.y) % 2 == 0 ? 0.2 : 0.1
    ), 1.);
    bool inBounds = texPos.x > 0 && texPos.x < 1 && texPos.y > 0 && texPos.y < 1;
    gl_FragColor = inBounds ? texColor : bgColor;
}
