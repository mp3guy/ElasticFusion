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

uniform sampler2D texVerts;
uniform float maxDepth;

in vec2 texcoord;

out vec4 FragColor;

void main()
{
    vec4 vertex = texture2D(texVerts, texcoord);
    
    if(vertex.z > maxDepth || vertex.z <= 0)
    {
        discard;
    }
    else
    {
        FragColor = 1.0f - vec4(vertex.z / maxDepth);
    }
}
