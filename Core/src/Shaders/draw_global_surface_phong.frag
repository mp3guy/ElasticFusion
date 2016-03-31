/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#version 330 core

in vec3 n;
in vec3 v;
in vec3 vColor0;
in vec2 texcoord;
in float radius;
flat in int unstablePoint;

out vec4 FragColor;

uniform vec3 lightpos;

void main()
{
    if(dot(texcoord, texcoord) > 1.0)
        discard;
    
    vec4 diffuse = vec4(0.0);
    vec4 specular = vec4(0.0);
    
    // ambient term
    vec4 ambient = vec4(0.3 * vColor0, 1);
    
    // diffuse color
    vec4 kd = vec4(vColor0, 1.0);
    
    // specular color
    vec4 ks = vec4(1.0, 1.0, 1.0, 1.0);
    
    // diffuse term
    vec3 lightDir = normalize(lightpos - v);
    float NdotL = dot(n, lightDir);
    
    if (NdotL > 0.0)
        diffuse = kd * NdotL;
    
    // specular term
    vec3 rVector = normalize(2.0 * n * dot(n, lightDir) - lightDir);
    vec3 viewVector = normalize(-v);
    float RdotV = dot(rVector, viewVector);
    
    if (RdotV > 0.0)
        specular = ks * pow(RdotV, 32);

    FragColor = ambient + diffuse + specular;
    
    if(unstablePoint == 1)
	{
		gl_FragDepth = gl_FragCoord.z + radius;
	}
    else
   	{
   		gl_FragDepth = gl_FragCoord.z;
   	}
}
