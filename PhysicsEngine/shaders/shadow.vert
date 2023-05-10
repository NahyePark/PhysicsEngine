
/////////////////////////////////////////////////////////////////////////
// Vertex shader for lighting
//
// Copyright 2013 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#version 330

uniform mat4 WorldView, WorldProj, ModelTr, NormalTr;

in vec4 vertex;
in vec3 vertexNormal;
in vec2 vertexTexture;
in vec3 vertexTangent;

out vec4 worldPos;
out vec4 position;

void main()
{
    gl_Position = WorldProj*WorldView*ModelTr*vertex;
    
    worldPos = ModelTr*vertex;

    position = gl_Position;
}