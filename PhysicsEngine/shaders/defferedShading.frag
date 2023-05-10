#version 330 core

#define MAX_LIGHTS 16

out vec4 FragColor;

in vec2 textCoord;

//uniform sampler2D gPosition;
//uniform sampler2D gNormal;
//uniform sampler2D gAlbedoSpec;
uniform int sphere_num;
uniform vec3 viewPos;

uniform struct LightArray{
	int type;
	//vec3 specularColor;
	//vec3 ambientColor;
	//vec3 diffuseColor;
	vec3 lightPosition; //point, spot
	vec3 lightDirection; //spot, directional
	vec3 lightColor;
	vec3 ambient_strength;
	
	float innerAngle;
	float outerAngle;
	float fallOff;
	
	float Linear;
    float Quadratic;
    float Radius;
}myLight[MAX_LIGHTS];



vec3 ApplyLight(LightArray light, vec3 N, vec3 fragPos, vec3 V,
				vec3 Diffuse, float Spec)
{


	vec3 L;
	float att = 0;
	vec3 normLightDir = normalize(light.lightDirection);
	float SpotlightEffect = 1.0f;
	
	if(light.type == 0)
	{//directional light
		L = normalize(light.lightPosition);
		att = 0.1f;
	}
	else if(light.type == 1)
	{
		//point light
		L = normalize(light.lightPosition - fragPos);
		float dD = length(light.lightPosition - fragPos);
		att = 1.0 / (1.0 + light.Linear * dD + 
  			     light.Quadratic * (dD * dD));  
				 
	}
		//spotlight
	else if(light.type == 2)
	{
		L = normalize(light.lightPosition - fragPos);
		float dD = length(light.lightPosition - fragPos);
		att = 1.0 / (1.0 + light.Linear * dD + 
  			     light.Quadratic * (dD * dD));  
	
		vec3 lightToVer = normalize(fragPos - light.lightPosition);
		
		float dotLD = dot(normLightDir, lightToVer);
		float outer = cos(light.outerAngle*3.14/180.f);
		float inner = cos(light.innerAngle*3.14/180.f);
		
		if( dotLD <= outer)
			SpotlightEffect = 0.0;
		else if(dotLD > inner)
			SpotlightEffect = 1.0;
		else
			SpotlightEffect = pow(((dotLD - outer)/(inner - outer)), light.fallOff);
	}
		
	
	//ambient
	vec3 ambient =  light.lightColor* light.ambient_strength; 
	
	//diffuse
	float N_dot_L = dot(N,L);
	float diff = max(N_dot_L, 0.0f);
	vec3 diffuse = diff * light.lightColor * Diffuse; 
	
	//Reflection vector
	vec3 R = (2*N_dot_L)*N-L;
	
	//specular
	float spec = pow(max(dot(V,R), 0.0f), 16);
	vec3 specular = spec * light.lightColor * Spec;
	
	ambient  *= att;
    diffuse  *= att*SpotlightEffect;
    specular *= att*SpotlightEffect;
	
	return (ambient + diffuse + specular);
	
	
}

void main()
{             
    // retrieve data from gbuffer
    vec3 FragPos = texture(gPosition, textCoord).rgb;
    vec3 Normal = texture(gNormal, textCoord).rgb;
    vec3 Diffuse = texture(gAlbedoSpec, textCoord).rgb;
    float Specular = texture(gAlbedoSpec, textCoord).a;
    
    // then calculate lighting as usual
    vec3 lighting  = Diffuse * 0.1;
    vec3 viewDir  = normalize(viewPos - FragPos);

    for(int i = 0; i < sphere_num; ++i)
    {
        float distance = length(myLight[i].lightPosition - FragPos);
        if(distance < myLight[i].Radius)
        {
			lighting += ApplyLight(myLight[i], Normal, FragPos, viewDir, Diffuse, Specular);
        }
    }

    FragColor = vec4(FragPos, 1.0f);
}