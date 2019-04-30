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

in vec2 texcoord;

out vec4 FragColor;

uniform sampler2D eSampler;
uniform usampler2D rSampler;
uniform vec4 cam; //cx, cy, 1/fx, 1/fy
uniform float cols;
uniform float rows;
uniform int passthrough;

#include "geometry.glsl"

void main()
{
    float halfPixX = 0.5 * (1.0 / cols);
    float halfPixY = 0.5 * (1.0 / rows);
    
     vec4 sample = textureLod(eSampler, texcoord, 0.0);
    
    if(sample.z == 0 || passthrough == 1)
    {
	    vec4 vPos = vec4(getVertex(texcoord, int(texcoord.x * cols), int(texcoord.y * rows), cam, rSampler), 1);
        FragColor = vec4(getNormal(vPos.xyz, texcoord, int(texcoord.x * cols), int(texcoord.y * rows), cam, rSampler), 1);
    }
    else
    {
        FragColor = sample;
    }
}
