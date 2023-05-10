/////////////////////////////////////////////////////////////////////////
// Pixel shader for lighting
////////////////////////////////////////////////////////////////////////
#version 330



float pi = 3.14159;
float pi2 = 2*pi;

out vec4 FragColor;

in vec2 texCoord;

uniform int width, height;
uniform vec3 lightPos, eyePos;
uniform mat4 shadowMatrix;
uniform float alpha;

uniform int gBuffer_num;
uniform int mode;
uniform vec3 lineColor;

uniform sampler2D gBufferWorldPos;
uniform sampler2D gBufferNormal;
uniform sampler2D gBufferDiffuse;
uniform sampler2D gBufferSpecular;
uniform sampler2D shadowMap;


vec3 cholesky(float m11 ,float m12 ,float m13 ,float m22 ,float m23 ,float  m33 , float  z1 ,float  z2 ,float  z3)
{
    float a = sqrt(m11);
    float b = m12 /a;
    float c = m13/a;
    float d = sqrt(m22-b*b);
    float e = (m23-b*c)/d;
    float f = sqrt(m33-c*c-e*e);
    float c1_hat = z1/a;
    float c2_hat =(z2-b*c1_hat)/d;
    float c3_hat =(z3-c*c1_hat-e*c2_hat)/ f;

    float c3 = c3_hat /f;
    float c2 =(c2_hat-e*c3)/d;
    float c1 =(c1_hat-b*c2-c*c3)/a;
    return vec3(c1,c2,c3);
}

void main()
{
    if(mode == 1) //debug
    {
        FragColor.xyz = lineColor;

    }
    else
    {
        vec2 uv = gl_FragCoord.xy / vec2(width, height);
        vec3 normalVec = texture2D(gBufferNormal, uv).xyz;
        vec3 Kd = texture2D(gBufferDiffuse, uv).xyz; 
        vec3 specular = texture2D(gBufferSpecular, uv).xyz; 
        float shininess = texture2D(gBufferSpecular, uv).w;
        vec4 worldPos = texture2D(gBufferWorldPos, uv);

        vec3 lightVec = lightPos - worldPos.xyz;
        vec3 eyeVec = eyePos - worldPos.xyz;
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

        vec4 shadowCoord = shadowMatrix * worldPos;
        vec2 shadowIndex = shadowCoord.xy/shadowCoord.w;

        //float lightDepth = texture2D(shadowMap, shadowIndex).x;
        float pixelDepth = shadowCoord.w/100;

        vec4 b = texture2D(shadowMap, shadowIndex);

        float shadow = 0;
        vec4 adjust = vec4(0.5,0.5,0.5,0.5);
        vec4 b_prime = (1-alpha)*b + alpha*adjust;

        vec3 c = cholesky(1, b_prime.x, b_prime.y, b_prime.y, b_prime.z, b_prime.w, 1, pixelDepth, pixelDepth*pixelDepth);
        c = c/c.z;

        float z2,z3;
        if(c.y*c.y - 4*c.z*c.x < 0)
        {
            z2 = (-c.y)/(2*c.z);
            z3 = z2;
        }
        else
        {
            z2 = (-c.y-sqrt(c.y*c.y - 4*c.z*c.x))/(2*c.z);
            z3 = (-c.y+sqrt(c.y*c.y - 4*c.z*c.x))/(2*c.z);
        }

    

        if(pixelDepth <= z2)
            shadow = 0;
        else if(pixelDepth <= z3)
        {
            shadow = (pixelDepth*z3 - b_prime.x*(pixelDepth+z3)+b_prime.y)/((z3-z2)*(pixelDepth-z2));
        }
        else
        {
             shadow = 1.0 - (z2*z3 - b_prime.x*(z2+z3)+b_prime.y)/((pixelDepth-z2)*(pixelDepth-z3));
        }

        //if(pixelDepth > lightDepth)
        //{
        //    shadow = 1.0;

        //}

        // A checkerboard pattern to break up larte flat expanses.  Remove when using textures.
        //if (objectId==groundId || objectId==floorId || objectId==seaId) {
        //    ivec2 uv = ivec2(floor(100.0*texCoord));
        //    if ((uv[0]+uv[1])%2==0)
        //        Kd *= 0.9; }

        if(gBuffer_num == 0)
            FragColor.xyz = Ia * Kd + I*Kd*NL + I*specular*pow(HN, shininess);
            //FragColor.xyz = Ia * Kd*(1.0f - shadow) + I*Kd*NL*(1.0f - shadow) + I*specular*pow(HN, shininess)*(1.0f - shadow);
        else if(gBuffer_num == 1)
            FragColor = worldPos;
        else if(gBuffer_num == 2)
            FragColor.xyz = normalVec;
        else if(gBuffer_num == 3)
            FragColor.xyz = Kd;
        else if(gBuffer_num == 4)
        {
            FragColor.xyz = specular;
            FragColor.w = shininess;
        }


       // Lighting is diffuse + ambient + specular
        //vec3 output = Ia*Kd;
        ///    output += I*Kd*NL;
         //   output += I*specular*pow(HN,shininess); 
        //FragColor.xyz = output;
    }
}
