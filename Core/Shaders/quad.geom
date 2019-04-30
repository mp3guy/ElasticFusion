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

out vec2 texcoord;

void main() 
{
    gl_Position = vec4(1.0, 1.0, 0.0, 1.0);
    texcoord = vec2(1.0, 1.0);
    EmitVertex();

    gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);
    texcoord = vec2(0.0, 1.0); 
    EmitVertex();

    gl_Position = vec4(1.0,-1.0, 0.0, 1.0);
    texcoord = vec2(1.0, 0.0); 
    EmitVertex();

    gl_Position = vec4(-1.0,-1.0, 0.0, 1.0);
    texcoord = vec2(0.0, 0.0); 
    EmitVertex();

    EndPrimitive(); 
}
