uniform vec3 color = vec3(1.0, 1.0, 1.0);
uniform sampler2D textureData;

in vec2 interpolatedTextureCoordinates;

out vec4 fragmentColor;

void main() {
    fragmentColor.rgb = color*texture(textureData, interpolatedTextureCoordinates).rgb;
    fragmentColor.a = 1.0;
}