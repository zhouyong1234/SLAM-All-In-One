#version 130

//! [0]
uniform sampler2D texture;

in vec2 varyingTextureCoordinate;

out vec4 fragColor;

void main(void)
{
    fragColor = texture2D(texture, varyingTextureCoordinate);
}
//! [0]
