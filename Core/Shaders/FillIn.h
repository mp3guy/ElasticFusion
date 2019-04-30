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

#ifndef FILLIN_H_
#define FILLIN_H_

#include "Shaders.h"
#include "Uniform.h"
#include "../Utils/Resolution.h"
#include "../Utils/Intrinsics.h"
#include "../GPUTexture.h"

class FillIn
{
    public:
        FillIn();
        virtual ~FillIn();

        void image(GPUTexture * existingRgb, GPUTexture * rawRgb, bool passthrough);
        void vertex(GPUTexture * existingVertex, GPUTexture * rawDepth, bool passthrough);
        void normal(GPUTexture * existingNormal, GPUTexture * rawDepth, bool passthrough);

        GPUTexture imageTexture;
        GPUTexture vertexTexture;
        GPUTexture normalTexture;

        std::shared_ptr<Shader> imageProgram;
        pangolin::GlRenderBuffer imageRenderBuffer;
        pangolin::GlFramebuffer imageFrameBuffer;

        std::shared_ptr<Shader> vertexProgram;
        pangolin::GlRenderBuffer vertexRenderBuffer;
        pangolin::GlFramebuffer vertexFrameBuffer;

        std::shared_ptr<Shader> normalProgram;
        pangolin::GlRenderBuffer normalRenderBuffer;
        pangolin::GlFramebuffer normalFrameBuffer;
};

#endif /* FILLIN_H_ */

