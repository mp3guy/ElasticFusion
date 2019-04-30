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

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform float threshold;
uniform float signMult;

in vec4 vColor[];
in vec4 vPosition[];
in vec4 vNormRad[];
in mat4 vMVP[];
in int colorType0[];
in int drawWindow0[];
in int vTime[];
in int timeDelta0[];

out vec3 vColor0;
out vec3 v;
out vec3 n;
out vec2 texcoord;
out float radius;
flat out int unstablePoint;

#include "color.glsl"

void main() 
{
    if(colorType0[0] != -1)
    {
        if(colorType0[0] == 1)
        {
            vColor0 = vNormRad[0].xyz;
        }
        else if(colorType0[0] == 2)
        {
            vColor0 = decodeColor(vColor[0].x);
        }
        else if(colorType0[0] == 3)
        {
            vColor0 = vec3(vColor[0].z / float(vTime[0]));
            
            float minimum = 1.0f;
            float maximum = float(vTime[0]);
            
            float ratio = 2 * (vColor[0].z - minimum) / (maximum - minimum);
            vColor0.x = max(0, (1 - ratio));
            vColor0.y = max(0, (ratio - 1));
            vColor0.z = 1.0f - vColor0.x - vColor0.y;
            
            vColor0.xyz *= abs(dot(vNormRad[0].xyz, vec3(1.0, 1.0, 1.0))) + vec3(0.1f, 0.1f, 0.1f);
        }
        else
        {
            vColor0 = (vec3(.5f, .5f, .5f) * abs(dot(vNormRad[0].xyz, vec3(1.0, 1.0, 1.0)))) + vec3(0.1f, 0.1f, 0.1f);
        }
    
        if(drawWindow0[0] == 1 && vTime[0] - vColor[0].w > timeDelta0[0])
        {
            vColor0 *= 0.25;
        }

		unstablePoint = (vPosition[0].w <= threshold ? 1 : 0);
		
		radius = vNormRad[0].w;
        
        vec3 x = normalize(vec3((vNormRad[0].y - vNormRad[0].z), -vNormRad[0].x, vNormRad[0].x)) * vNormRad[0].w * 1.41421356;
        
        vec3 y = cross(vNormRad[0].xyz, x);

        n = signMult * vNormRad[0].xyz;

        texcoord = vec2(-1.0, -1.0);
        gl_Position = vMVP[0] * vec4(vPosition[0].xyz + x, 1.0);
        v = vPosition[0].xyz + x;
        EmitVertex();

        texcoord = vec2(1.0, -1.0);
        gl_Position = vMVP[0] * vec4(vPosition[0].xyz + y, 1.0);
        v = vPosition[0].xyz + y;
        EmitVertex();

        texcoord = vec2(-1.0, 1.0);
        gl_Position = vMVP[0] * vec4(vPosition[0].xyz - y, 1.0);
        v = vPosition[0].xyz - y;
        EmitVertex();

        texcoord = vec2(1.0, 1.0);
        gl_Position = vMVP[0] * vec4(vPosition[0].xyz - x, 1.0);
        v = vPosition[0].xyz - x;
        EmitVertex();
        EndPrimitive();
    }
}
