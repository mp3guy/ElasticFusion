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

#ifndef COMPUTEPACK_H_
#define COMPUTEPACK_H_

#include "Shaders.h"
#include "../Utils/Resolution.h"
#include "Uniform.h"
#include <pangolin/gl/gl.h>

class ComputePack
{
    public:
        ComputePack(std::shared_ptr<Shader> program,
                    pangolin::GlTexture * target);

        virtual ~ComputePack();

        static const std::string NORM, FILTER, METRIC, METRIC_FILTERED;

        void compute(pangolin::GlTexture * input, const std::vector<Uniform> * const uniforms = 0);

    private:
        std::shared_ptr<Shader> program;
        pangolin::GlRenderBuffer renderBuffer;
        pangolin::GlTexture * target;
        pangolin::GlFramebuffer frameBuffer;
};

#endif /* COMPUTEPACK_H_ */
