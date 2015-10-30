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

uniform vec4 cam; //cx, cy, fx, fy
uniform float maxDepth;

in vec4 position;
in vec4 normRad;
in vec4 colTime;

layout(location = 0) out vec4 image;
layout(location = 1) out vec4 vertex;
layout(location = 2) out vec4 normal;
layout(location = 3) out uint time;

#include "color.glsl"

void main()
{
    vec3 l = normalize(vec3((vec2(gl_FragCoord) - cam.xy) / cam.zw, 1.0f));
    
    vec3 corrected_pos = (dot(position.xyz, normRad.xyz) / dot(l, normRad.xyz)) * l; 

    //check if the intersection is inside the surfel
    float sqrRad = pow(normRad.w, 2);
    vec3 diff = corrected_pos - position.xyz;

    if(dot(diff, diff) > sqrRad)
    {
        discard;
    }

    image = vec4(decodeColor(colTime.x), 1);
    
    float z = corrected_pos.z;
    
    vertex = vec4((gl_FragCoord.x - cam.x) * z * (1.f / cam.z), (gl_FragCoord.y - cam.y) * z * (1.f / cam.w), z, position.w);
    
    normal = normRad;
    
    time = uint(colTime.z);
    
    gl_FragDepth = (corrected_pos.z / (2 * maxDepth)) + 0.5f;
}
