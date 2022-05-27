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
flat out int id;

void main()
{
    vPosition0 = vPosition;
    vColorTime0 = vColorTime;
    vNormRad0 = vNormRad;
    id = gl_VertexID;    
}
