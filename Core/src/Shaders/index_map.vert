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

layout (location = 0) in vec4 vPosition;
layout (location = 1) in vec4 vColorTime;
layout (location = 2) in vec4 vNormRad;

out vec4 vPosition0;
out vec4 vColorTime0;
out vec4 vNormRad0;
flat out int vertexId;

uniform mat4 t_inv;
uniform vec4 cam; //cx, cy, fx, fy
uniform float cols;
uniform float rows;
uniform float maxDepth;
uniform int time;
uniform int timeDelta;

void main()
{
    vec4 vPosHome = t_inv * vec4(vPosition.xyz, 1.0);
    
    float x = 0;
    float y = 0;
        
    if(vPosHome.z > maxDepth || vPosHome.z < 0 || time - vColorTime.w > timeDelta)
    {
        x = -10;
        y = -10;
        vertexId = 0;
    }
    else
    {
        x = ((((cam.z * vPosHome.x) / vPosHome.z) + cam.x) - (cols * 0.5)) / (cols * 0.5);
        y = ((((cam.w * vPosHome.y) / vPosHome.z) + cam.y) - (rows * 0.5)) / (rows * 0.5);
        vertexId = gl_VertexID;
    }
    
    gl_Position = vec4(x, y, vPosHome.z / maxDepth, 1.0);

    vPosition0 = vec4(vPosHome.xyz, vPosition.w);
    vColorTime0 = vColorTime;
    vNormRad0 = vec4(normalize(mat3(t_inv) * vNormRad.xyz), vNormRad.w);
}
