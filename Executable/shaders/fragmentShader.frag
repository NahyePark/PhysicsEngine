#version 330 core
out vec4 FragColor;
  
// These definitions agree with the ObjectIds enum in scene.h
const int     nullId	= 0;
const int     skyId	= 1;
const int     seaId	= 2;
const int     groundId	= 3;
const int     roomId	= 4;
const int     boxId	= 5;
const int     frameId	= 6;
const int     lPicId	= 7;
const int     rPicId	= 8;
const int     teapotId	= 9;
const int     spheresId	= 10;
const int     floorId	= 11;
  

in vec3 normalVec, lightVec, eyeVec;
in vec3 colorVec;
in vec2 texCoord;

uniform int objectId;  
uniform vec3 diffuse;
uniform vec3 specular;
uniform float shininess;

uniform sampler2D ourTexture;

void main()
{
	if(objectId == teapotId)
		FragColor = texture(ourTexture, texCoord);
	else
	{
		vec3 ONE = vec3(1.0, 1.0, 1.0);
		vec3 N = normalize(normalVec);
		vec3 L = normalize(lightVec);
		vec3 V = normalize(eyeVec);
		vec3 H = normalize(L+V);
		float NL = max(dot(N,L),0.0);
		float NV = max(dot(N,V),0.0);
		float HN = max(dot(H,N),0.0);

		vec3 I = ONE;
		vec3 Ia = 0.2*ONE;
		vec3 Kd = diffuse; 
		
		// A checkerboard pattern to break up larte flat expanses.  Remove when using textures.
		if (objectId==groundId || objectId==floorId || objectId==seaId) {
			ivec2 uv = ivec2(floor(100.0*texCoord));
			if ((uv[0]+uv[1])%2==0)
				Kd *= 0.9; }
		
	   // Lighting is diffuse + ambient + specular
		vec3 output = Ia*Kd;
			output += I*Kd*NL;
			output += I*specular*pow(HN,shininess); 
		FragColor.xyz = output;
	}
}