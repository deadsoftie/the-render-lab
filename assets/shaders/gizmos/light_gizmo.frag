#version 330 core
in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uIcon;
uniform vec3 uColor;

void main()
{
    vec4 icon = texture(uIcon, vUV);

    // Alpha clip for clean edges
    if (icon.a < 0.1)
        discard;

    FragColor = vec4(icon.rgb * uColor, icon.a);
}
