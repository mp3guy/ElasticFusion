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

in vec4 vPosition0;
in vec4 vColor0;
in vec4 vNormRad0;
flat in int updateId0;

layout(location = 0) out vec4 vPosition1;
layout(location = 1) out vec4 vColor1;
layout(location = 2) out vec4 vNormRad1;

void main() 
{
    //If we have a point to update in the existing model, store that
    if(updateId0 == 1)
    {
        vPosition1 = vPosition0;
        vColor1 = vColor0;
        vNormRad1 = vNormRad0;
    }
}
