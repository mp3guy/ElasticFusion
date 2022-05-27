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
layout(points, max_vertices = 1) out;

in vec4 vPosition[];
in vec4 vColor[];
in vec4 vNormRad[];
flat in int updateId[];

out vec4 vPosition0;
out vec4 vColor0;
out vec4 vNormRad0;
flat out int updateId0;

void main() 
{
    //Emit a vertex if either we have an update to store, or a new unstable vertex to store
    if(updateId[0] > 0)
    {
	    vPosition0 = vPosition[0];
	    vColor0 = vColor[0];
	    vNormRad0 = vNormRad[0];
	    updateId0 = updateId[0];
	    
	    //This will be -10, -10 (offscreen) for new unstable vertices, so they don't show in the fragment shader
	    gl_Position = gl_in[0].gl_Position;

	    EmitVertex();
	    EndPrimitive(); 
    }
}
