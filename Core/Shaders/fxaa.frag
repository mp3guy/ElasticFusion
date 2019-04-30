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

#define FXAA_REDUCE_MIN   (1.0/ 128.0)
#define FXAA_REDUCE_MUL   (1.0 / 8.0)
#define FXAA_SPAN_MAX     8.0

uniform sampler2D tex;
uniform vec2 resolution;

in vec2 texcoord;

out vec4 FragColor;

void main()
{
    vec4 color;
    
    vec2 inverseVP = 1.0 / resolution.xy;
    vec2 v_rgbNW = texcoord + (vec2(-1.0, -1.0) * inverseVP);
    vec2 v_rgbNE = texcoord + (vec2(1.0, -1.0) * inverseVP);
    vec2 v_rgbSW = texcoord + (vec2(-1.0, 1.0) * inverseVP);
    vec2 v_rgbSE = texcoord + (vec2(1.0, 1.0) * inverseVP);
    
    vec2 v_rgbN = texcoord + (vec2(-1.0, 0.0) * inverseVP);
    vec2 v_rgbE = texcoord + (vec2(1.0, 0.0) * inverseVP);
    vec2 v_rgbW = texcoord + (vec2(0.0, -1.0) * inverseVP);
    vec2 v_rgbS = texcoord + (vec2(0.0, 1.0) * inverseVP);
    
    vec2 v_rgbM = texcoord;
    
    vec3 rgbNW = texture2D(tex, v_rgbNW).xyz;
    vec3 rgbNE = texture2D(tex, v_rgbNE).xyz;
    vec3 rgbSW = texture2D(tex, v_rgbSW).xyz;
    vec3 rgbSE = texture2D(tex, v_rgbSE).xyz;
    
    vec3 rgbN = texture2D(tex, v_rgbN).xyz;
    vec3 rgbE = texture2D(tex, v_rgbE).xyz;
    vec3 rgbW = texture2D(tex, v_rgbW).xyz;
    vec3 rgbS = texture2D(tex, v_rgbS).xyz;
    
    vec3 rgbM  = texture2D(tex, v_rgbM).xyz;
    vec3 luma = vec3(0.299, 0.587, 0.114);
    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM  = dot(rgbM,  luma);
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    
    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));
    
    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);
    
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);
    
    dir = min(vec2(FXAA_SPAN_MAX, FXAA_SPAN_MAX), max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin)) * inverseVP;
    
    vec3 rgbA = 0.5 * (texture2D(tex, texcoord + dir * (1.0 / 3.0 - 0.5)).xyz + texture2D(tex, texcoord + dir * (2.0 / 3.0 - 0.5)).xyz);
    
    vec3 rgbB = rgbA * 0.5 + 0.25 * (texture2D(tex, texcoord + dir * -0.5).xyz + texture2D(tex, texcoord + dir * 0.5).xyz);

    float lumaB = dot(rgbB, luma);
    
    if ((lumaB < lumaMin) || (lumaB > lumaMax))
        FragColor = vec4(rgbA, 1);
    else
        FragColor = vec4(rgbB, 1);
}
