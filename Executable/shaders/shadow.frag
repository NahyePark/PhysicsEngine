in vec4 worldPos;
in vec4 position;
uniform float farPlane;

void main()
{   
	float z = position.w/farPlane;
	gl_FragData[0].x = z;
	gl_FragData[0].y = z*z;
	gl_FragData[0].z = z*z*z;
	gl_FragData[0].w = z*z*z*z;
}