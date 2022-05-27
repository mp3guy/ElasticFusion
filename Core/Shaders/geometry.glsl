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

//Central difference on floating point depth maps
//Cam is //cx, cy, 1 / fx, 1 / fy
vec3 getVertex(vec2 texCoord, float x, float y, vec4 cam, sampler2D depth)
{
    float z = float(textureLod(depth, texCoord, 0.0));
    return vec3((x - cam.x) * z * cam.z, (y - cam.y) * z * cam.w, z);
}

//Cam is //cx, cy, 1 / fx, 1 / fy
vec3 getNormal(vec3 vPosition, vec2 texCoord, float x, float y, vec4 cam, sampler2D depth)
{
    vec3 vPosition_xf = getVertex(vec2(texCoord.x + (1.0 / cols), texCoord.y), x + 1, y, cam, depth);
    vec3 vPosition_xb = getVertex(vec2(texCoord.x - (1.0 / cols), texCoord.y), x - 1, y, cam, depth);
    
    vec3 vPosition_yf = getVertex(vec2(texCoord.x, texCoord.y + (1.0 / rows)), x, y + 1, cam, depth);
    vec3 vPosition_yb = getVertex(vec2(texCoord.x, texCoord.y - (1.0 / rows)), x, y - 1, cam, depth);
    
    vec3 del_x = ((vPosition_xb + vPosition) / 2) - ((vPosition_xf + vPosition) / 2);
    vec3 del_y = ((vPosition_yb + vPosition) / 2) - ((vPosition_yf + vPosition) / 2);
    
    return normalize(cross(del_x, del_y));
}

//Forward difference on raw depth maps still in ushort mm
//Cam is //cx, cy, 1 / fx, 1 / fy
vec3 getVertex(vec2 texcoord, int x, int y, vec4 cam, usampler2D depth)
{
    float z = float(textureLod(depth, texcoord, 0.0)) / 1000.0f;
    return vec3((x - cam.x) * z * cam.z, (y - cam.y) * z * cam.w, z);
}

//Cam is //cx, cy, 1 / fx, 1 / fy
vec3 getNormal(vec3 vPosition, vec2 texcoord, int x, int y, vec4 cam, usampler2D depth)
{
    vec3 vPosition_x = getVertex(vec2(texcoord.x + (1.0 / cols), texcoord.y), x + 1, y, cam, depth);
    vec3 vPosition_y = getVertex(vec2(texcoord.x, texcoord.y + (1.0 / rows)), x, y + 1, cam, depth);
    
    vec3 del_x = vPosition_x - vPosition;
    vec3 del_y = vPosition_y - vPosition;
    
    return normalize(cross(del_x, del_y));
}
