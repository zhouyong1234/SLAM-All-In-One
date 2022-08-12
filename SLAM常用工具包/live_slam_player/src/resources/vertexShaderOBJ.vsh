#version 130

//! [0]
uniform mat4 mvpMatrix;

in vec4 vertex;
in vec2 textureCoordinate;

out vec2 varyingTextureCoordinate;

void main(void)
{
    varyingTextureCoordinate = textureCoordinate;
    gl_Position = mvpMatrix * vertex;
}
//! [0]
