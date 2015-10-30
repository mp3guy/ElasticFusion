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

out float FragColor;

uniform usampler2D gSampler;
uniform float maxD;

void main()
{
    uint value = uint(texture(gSampler, texcoord.xy));
    
    if(value > uint(maxD * 1000.0f) || value < 300U)
    {
        FragColor = 0U;
    }
    else
    {
	    FragColor = float(value) / 1000.0f;
    }
}
