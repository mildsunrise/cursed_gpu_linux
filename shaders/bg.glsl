#version 330 core

uniform float bgCellSize;
uniform vec2 bgCellOffset;

void main() {
    ivec2 cell = ivec2(floor((gl_FragCoord.xy - bgCellOffset) / bgCellSize));
    gl_FragColor = vec4(vec3(
        (cell.x + cell.y) % 2 == 0 ? 0.2 : 0.1
    ), 1.);
}
