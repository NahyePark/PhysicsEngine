#version 330 core
in vec3 vertex;
in vec2 vertexTexture;

out vec2 textCoord;

void main()
{
	gl_Position = vec4(vertex, 1.0);
	textCoord = vertexTexture;
}