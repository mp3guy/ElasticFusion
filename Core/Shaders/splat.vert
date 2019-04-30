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
layout (location = 1) in vec4 vColor;
layout (location = 2) in vec4 vNormRad;

uniform mat4 t_inv;
uniform vec4 cam; //cx, cy, fx, fy
uniform float cols;
uniform float rows;
uniform float maxDepth;
uniform float confThreshold;
uniform int time;
uniform int maxTime;
uniform int timeDelta;

out vec4 position;
out vec4 normRad;
out vec4 colTime;

vec3 projectPoint(vec3 p)
{
    return vec3(((((cam.z * p.x) / p.z) + cam.x) - (cols * 0.5)) / (cols * 0.5),
                ((((cam.w * p.y) / p.z) + cam.y) - (rows * 0.5)) / (rows * 0.5),
                p.z / maxDepth);
}

vec3 projectPointImage(vec3 p)
{
    return vec3(((cam.z * p.x) / p.z) + cam.x,
                ((cam.w * p.y) / p.z) + cam.y,
                p.z);
}

void main()
{
    vec4 vPosHome = t_inv * vec4(vPosition.xyz, 1.0);
    
    if(vPosHome.z > maxDepth || vPosHome.z < 0 || vPosition.w < confThreshold || time - vColor.w > timeDelta || vColor.w > maxTime)
    {
        gl_Position = vec4(1000.0f, 1000.0f, 1000.0f, 1000.0f);
        gl_PointSize = 0;
    }
    else
    {
	    gl_Position = vec4(projectPoint(vPosHome.xyz), 1.0);
	    
        colTime = vColor;
	    position = vec4(vPosHome.xyz, vPosition.w);
	    normRad = vec4(normalize(mat3(t_inv) * vNormRad.xyz), vNormRad.w);
	    
	    vec3 x1 = normalize(vec3((normRad.y - normRad.z), -normRad.x, normRad.x)) * normRad.w * 1.41421356;
	    
	    vec3 y1 = cross(normRad.xyz, x1);
	
	    vec4 proj1 = vec4(projectPointImage(vPosHome.xyz + x1), 1.0);
	    vec4 proj2 = vec4(projectPointImage(vPosHome.xyz + y1), 1.0);
	    vec4 proj3 = vec4(projectPointImage(vPosHome.xyz - y1), 1.0);
	    vec4 proj4 = vec4(projectPointImage(vPosHome.xyz - x1), 1.0);
	                
	    vec2 xs = vec2(min(proj1.x, min(proj2.x, min(proj3.x, proj4.x))), max(proj1.x, max(proj2.x, max(proj3.x, proj4.x))));
	    vec2 ys = vec2(min(proj1.y, min(proj2.y, min(proj3.y, proj4.y))), max(proj1.y, max(proj2.y, max(proj3.y, proj4.y))));
	
	    float xDiff = abs(xs.y - xs.x);
	    float yDiff = abs(ys.y - ys.x);
	
	    gl_PointSize = max(0, max(xDiff, yDiff));
    }
}
