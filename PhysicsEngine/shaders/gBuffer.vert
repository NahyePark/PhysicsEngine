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

out vec3 normalVec;
out vec2 texCoord;
out vec4 worldPos;

void main()
{
    gl_Position = WorldProj*WorldView*ModelTr*vertex;
    
    worldPos = ModelTr*vertex;
    
    normalVec = vertexNormal*mat3(NormalTr); 

    texCoord = vertexTexture; 
}
